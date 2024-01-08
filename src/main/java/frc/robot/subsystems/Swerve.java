package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SerialPort;
import frc.robot.Constants;

import org.littletonrobotics.junction.Logger;

import com.kauailabs.navx.frc.AHRS;

public class Swerve extends SubsystemBase {
    private final java.util.logging.Logger logger;

    private final SwerveModule frontLeft;
    private final SwerveModule frontRight;
    private final SwerveModule backLeft;
    private final SwerveModule backRight;

    private SwerveModuleState[] lastIntendedStates;


    private AHRS gyro = new AHRS(SerialPort.Port.kUSB);
    private SwerveDriveOdometry odometry = new SwerveDriveOdometry(Constants.RobotDimensions.SWERVE_DRIVE_KINEMATICS,
            new Rotation2d(0), new SwerveModulePosition[] {new SwerveModulePosition(), new SwerveModulePosition(),
                    new SwerveModulePosition(), new SwerveModulePosition()});

    public Swerve() {
        logger = java.util.logging.Logger.getLogger(Swerve.class.getName());

        frontLeft = new SwerveModule("Front Left", Constants.Motor.SWERVE_FRONT_LEFT_POWER,
                Constants.Motor.SWERVE_FRONT_LEFT_STEER, 0);
        frontRight = new SwerveModule("Front Right", Constants.Motor.SWERVE_FRONT_RIGHT_POWER,
                Constants.Motor.SWERVE_FRONT_RIGHT_STEER, 0);
        backLeft = new SwerveModule("Back Left", Constants.Motor.SWERVE_BACK_LEFT_POWER,
                Constants.Motor.SWERVE_BACK_LEFT_STEER, 0);
        backRight = new SwerveModule("Back Right", Constants.Motor.SWERVE_BACK_RIGHT_POWER,
                Constants.Motor.SWERVE_BACK_RIGHT_STEER, 0);
        gyro.reset();
        gyro.resetDisplacement();

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

    public void setDrivebaseWheelVectors(double xSpeed, double ySpeed, double rotationSpeed, boolean fieldOriented,
            boolean forScoring) {
        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotationSpeed,
                fieldOriented ? getRotation2d() : new Rotation2d());
        SwerveModuleState[] moduleStates =
                Constants.RobotDimensions.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
        if (forScoring) {
            SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.Swerve.SWERVE_MAX_AUTO_SPEED);
        } else {
            SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.Swerve.SWERVE_MAX_SPEED);
        }
        setModuleStates(moduleStates);
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        lastIntendedStates = desiredStates;

        frontLeft.setState(desiredStates[0]);
        frontRight.setState(desiredStates[1]);
        backLeft.setState(desiredStates[2]);
        backRight.setState(desiredStates[3]);
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
        return Rotation2d.fromDegrees(gyro.getYaw() + 90);
    }

    public Pose2d getOdometryPose() {
        return odometry.getPoseMeters();
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
        return new SwerveModulePosition[] {frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(),
                backRight.getPosition()};
    }

    @Override
    public void periodic() {
        odometry.update(gyro.getRotation2d(), new SwerveModulePosition[] {frontLeft.getPosition(),
                frontRight.getPosition(), backLeft.getPosition(), backRight.getPosition()});

        Logger.recordOutput("SwerveDrive/IntendedStates",
                lastIntendedStates == null ? new SwerveModuleState[] {} : lastIntendedStates);
        Logger.recordOutput("SwerveDrive/GyroscopeHeading", getRotation2d().getDegrees());
        Logger.recordOutput("Gyro/Pitch", gyro.getPitch());
        Logger.recordOutput("Gyro/Yaw", gyro.getYaw());
        Logger.recordOutput("Gyro/Roll", gyro.getRoll());
        Logger.recordOutput("Gyro/xDisplacement", getDisplacementX());
        Logger.recordOutput("Gyro/yDisplacement", getDisplacementY());
        Logger.recordOutput("Gyro/zDisplacement", getDisplacementZ());
    }
}
