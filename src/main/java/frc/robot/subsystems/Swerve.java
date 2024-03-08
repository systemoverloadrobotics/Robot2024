package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.assistants.LimelightHelpers;
import frc.robot.assistants.PoseEstimatorHelper;
import frc.robot.assistants.LimelightHelpers.LimelightResults;
import frc.robot.assistants.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.assistants.LimelightHelpers.Results;

import java.util.Arrays;
import java.util.Optional;
import java.util.logging.Level;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

public class Swerve extends SubsystemBase {
    private final java.util.logging.Logger logger;

    private final SwerveModule frontLeft;
    private final SwerveModule frontRight;
    private final SwerveModule backLeft;
    private final SwerveModule backRight;

    private SwerveModuleState[] lastIntendedStates;

    private Pose2d lastPoseState;

    private AHRS gyro = new AHRS(SerialPort.Port.kUSB);
    
    private SwerveDrivePoseEstimator odometry = new SwerveDrivePoseEstimator(
        Constants.RobotDimensions.SWERVE_DRIVE_KINEMATICS,
        new Rotation2d(0),
        new SwerveModulePosition[] {new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()},
        new Pose2d(),
        VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
        VecBuilder.fill(0.0334, 0.1391, Units.degreesToRadians(30))
    );

    private SwerveDriveOdometry odometryReal = new SwerveDriveOdometry(
        Constants.RobotDimensions.SWERVE_DRIVE_KINEMATICS,
        new Rotation2d(0),
        new SwerveModulePosition[] {new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()}
    );

    public Swerve() {
        logger = java.util.logging.Logger.getLogger(Swerve.class.getName());
        lastIntendedStates = new SwerveModuleState[] {};

        frontLeft = new SwerveModule("Front Left", Constants.Motor.SWERVE_FRONT_LEFT_POWER,
                Constants.Motor.SWERVE_FRONT_LEFT_STEER, 0, true, false);
        frontRight = new SwerveModule("Front Right", Constants.Motor.SWERVE_FRONT_RIGHT_POWER,
                Constants.Motor.SWERVE_FRONT_RIGHT_STEER, 0, true, false);
        backLeft = new SwerveModule("Back Left", Constants.Motor.SWERVE_BACK_LEFT_POWER,
                Constants.Motor.SWERVE_BACK_LEFT_STEER, 0, true, false);
        backRight = new SwerveModule("Back Right", Constants.Motor.SWERVE_BACK_RIGHT_POWER,
                Constants.Motor.SWERVE_BACK_RIGHT_STEER, 0, true, false);
        gyro.reset();
        gyro.resetDisplacement();

        lastPoseState = new Pose2d();
        
        AutoBuilder.configureHolonomic(
                this::getOdometryPose, // Robot pose supplier
                this::resetOdometryPose, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (x) -> this.setDrivebaseWheelVectors(x, false), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                        4, // Max module speed, in m/s
                        0.3556 * Math.sqrt(2), // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
                ),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );

        logger.info("Swerve Drive Initialized.");
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {frontLeft.getState(), frontRight.getState(), backLeft.getState(),
                backRight.getState()};
    }

    public void resetPoseWithLimelight() {
        odometry.resetPosition(getRotation2d(), getModulePositions(), PoseEstimatorHelper.getEstimatedGlobalPose(lastPoseState));
    }

    public void setDrivebaseWheelVectors(ChassisSpeeds chassisSpeeds, boolean forScoring) {
        ChassisSpeeds.discretize(chassisSpeeds, 0.02);
        SwerveModuleState[] moduleStates =
                Constants.RobotDimensions.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
        if (forScoring) {
            SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.Swerve.SWERVE_MAX_AUTO_SPEED);
        } else {
            SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.Swerve.SWERVE_MAX_SPEED);
        }
        Logger.recordOutput("SwerveDrive/IntendedStatetws", moduleStates);
        setModuleStates(moduleStates);
    }

    public void setDrivebaseWheelVectors(double xSpeed, double ySpeed, double rotationSpeed, boolean fieldOriented, boolean forScoring) {
        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotationSpeed, fieldOriented ? getRotation2d() : new Rotation2d());
        setDrivebaseWheelVectors(chassisSpeeds, forScoring);
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        lastIntendedStates = desiredStates;

        frontLeft.setState(desiredStates[0]);
        frontRight.setState(desiredStates[1]);
        backLeft.setState(desiredStates[2]);
        backRight.setState(desiredStates[3]);
    }

    public ChassisSpeeds getChassisSpeeds() {
        return Constants.RobotDimensions.SWERVE_DRIVE_KINEMATICS.toChassisSpeeds(lastIntendedStates);
    }

    public void lock() {
        setModuleStates(new SwerveModuleState[] {new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),});
    }

    public void reset() {
        setModuleStates(new SwerveModuleState[] {new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(0)),});
    }

    // TODO ADJUST ANGLE OFFSET

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(gyro.getYaw());
    }

    public Pose2d getOdometryPose() {
        return odometry.getEstimatedPosition();
    }

    public void resetOdometryPose(Pose2d pose) {
        odometry.resetPosition(getRotation2d(), getModulePositions(), pose);
    }

    public double getPitch() {
        return gyro.getPitch();
    }

    public double getDisplacementX() {
        return gyro.getDisplacementX();
    }

    public double getDisplacementY() {
        return gyro.getDisplacementY();
    }

    public double getDisplacementZ() {
        return gyro.getDisplacementZ();
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(), backRight.getPosition()};
    }

    public void updateOdometry() {
        odometry.updateWithTime(Timer.getFPGATimestamp(), gyro.getRotation2d(), getModulePositions());
        odometryReal.update(gyro.getRotation2d(), getModulePositions()); // pure odometry
        
        LimelightResults result = LimelightHelpers.getLatestResults("limelight-lemon");
        if (result.error.equals("")) {
            LimelightTarget_Fiducial[] targets = result.targetingResults.targets_Fiducials;
            
            if (Arrays.stream(targets).anyMatch(x -> x.fiducialID == 4 || x.fiducialID == 8)) { // SHOOTER AIM CODE
                LimelightTarget_Fiducial target = Arrays.stream(targets).filter(x -> x.fiducialID == 4 || x.fiducialID == 8).findFirst().get(); // check if its the shooter area
                double distance = target.getTargetPose_RobotSpace2D().getTranslation().getNorm(); // Meters
                PoseEstimatorHelper.angleShootEstimate = Pivot.getAngleNeeded(Units.metersToFeet(distance));
            }

            if (targets.length > 0) { // if tags viewed
                double avgDistanceAway = Arrays.stream(targets).mapToDouble(x -> x.getTargetPose_CameraSpace2D().getTranslation().getNorm()).sum();
                Pose2d poseUpdatedGyro = new Pose2d(result.targetingResults.getBotPose2d_wpiBlue().getTranslation(), gyro.getRotation2d());
                double timestamp = Timer.getFPGATimestamp() - (result.targetingResults.latency_capture/1000.0) - (result.targetingResults.latency_pipeline/1000.0);

                double stdForVision = PoseEstimatorHelper.visionStdLookup.get(avgDistanceAway / targets.length);

                try {
                    odometry.addVisionMeasurement(
                        poseUpdatedGyro, 
                        timestamp,
                        VecBuilder.fill(stdForVision, stdForVision, Units.degreesToRadians(360)) // discard angle measurement
                    );
                } catch (Exception e) {
                    logger.log(Level.SEVERE, e.toString());
                }
            } 
        }

        Logger.recordOutput("Odometry", odometry.getEstimatedPosition());
        Logger.recordOutput("PureOdometry", odometryReal.getPoseMeters());
    }

    @Override
    public void periodic() {
        updateOdometry();

        Logger.recordOutput("SwerveDrive/IntendedStates",
                getModulePositions() == null ? new SwerveModuleState[] {} : getModulePositions());
        Logger.recordOutput("SwerveDrive/GyroscopeHeading", getRotation2d().getDegrees());
        Logger.recordOutput("Gyro/Pitch", gyro.getPitch());
        Logger.recordOutput("Gyro/Yaw", gyro.getYaw());
        Logger.recordOutput("Gyro/Roll", gyro.getRoll());
        Logger.recordOutput("Gyro/xDisplacement", getDisplacementX());
        Logger.recordOutput("Gyro/yDisplacement", getDisplacementY());
        Logger.recordOutput("Gyro/zDisplacement", getDisplacementZ());
    }
}
