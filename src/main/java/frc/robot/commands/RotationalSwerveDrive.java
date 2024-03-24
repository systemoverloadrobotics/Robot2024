package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.sorutil.SorMath;


public class RotationalSwerveDrive extends Command {
    protected final Swerve swerve;
    protected SlewRateLimiter xLimiter, yLimiter, rotationLimiter;

    protected final DoubleSupplier xSupplier, ySupplier, rotationPositionSupplier;
    protected final BooleanSupplier lockSupplier;

    private final ProfiledPIDController angleController = new ProfiledPIDController(0.5, 0, 0.025, new TrapezoidProfile.Constraints(3600, 3600));

    public RotationalSwerveDrive(Swerve swerve, DoubleSupplier xSupplier, DoubleSupplier ySupplier,
            DoubleSupplier rotationPositionSupplier, BooleanSupplier lockSupplier) {
        this.swerve = swerve;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.rotationPositionSupplier = rotationPositionSupplier;
        this.lockSupplier = lockSupplier;

        xLimiter = new SlewRateLimiter(Constants.Swerve.SWERVE_MAX_SPEED);
        yLimiter = new SlewRateLimiter(Constants.Swerve.SWERVE_MAX_SPEED);
        rotationLimiter = new SlewRateLimiter(Constants.Swerve.SWERVE_ROTATION_MAX_SPEED);

        angleController.setTolerance(Constants.Swerve.SWERVE_ROTATION_TOLERANCE);
        angleController.enableContinuousInput(0, 360);

        addRequirements(swerve);
    }

    protected double cleanAndScaleInput(double input, SlewRateLimiter limiter, double speedScaling) {
        input = (Math.abs(input) > Constants.Swerve.SWERVE_DEADBAND) ? input : 0;
        input = SorMath.signedSquare(input);
        input = limiter.calculate(input);
        input *= speedScaling;

        return input;
    }

    // Called at 50hz while the command is scheduled.
    @Override
    public void execute() {
        if (lockSupplier.getAsBoolean()) {
            swerve.lock();
            return;
        }
        Logger.recordOutput("Hi", 1);
        // get joystick inputs and clean/scale them
        double xSpeed = cleanAndScaleInput(xSupplier.getAsDouble(), xLimiter, Constants.Swerve.SWERVE_MAX_SPEED);
        double ySpeed = cleanAndScaleInput(ySupplier.getAsDouble(), yLimiter, Constants.Swerve.SWERVE_MAX_SPEED);
        double rotationSpeed = angleController.calculate(swerve.getRotation2d().getDegrees(), rotationPositionSupplier.getAsDouble());
        
        swerve.setDrivebaseWheelVectors(xSpeed, ySpeed, rotationSpeed, true, false);
        executeLogging(xSpeed, ySpeed, rotationSpeed);
    }

    protected void executeLogging(double xSpeed, double ySpeed, double rotationSpeed) {
        Logger.recordOutput("RSwerveDrive/xSpeed", xSpeed);
        Logger.recordOutput("RSwerveDrive/ySpeed", ySpeed);
        Logger.recordOutput("RSwerveDrive/inputX", xSupplier.getAsDouble());
        Logger.recordOutput("RSwerveDrive/inputY", ySupplier.getAsDouble());
    //    Logger.getInstance().recordOutput("SwerveDrive/inputR", rotationSupplier.getAsDouble());
        SmartDashboard.putNumber("RRRRrspeed", rotationSpeed);
        SmartDashboard.putNumber("RRRRrotation 2d", swerve.getRotation2d().getDegrees());
        Logger.recordOutput("RSwerveDrive/rotation", swerve.getRotation2d().getDegrees());
    }

    // Called once when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        swerve.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}