package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.sorutil.SorMath;


public class AutonSwerveDrive extends Command {
    protected final Swerve swerve;
    protected SlewRateLimiter xLimiter, yLimiter, rotationLimiter;

    protected final DoubleSupplier xSupplier, ySupplier, rotationSupplier;
    protected final BooleanSupplier lockSupplier;

    public AutonSwerveDrive(Swerve swerve, DoubleSupplier xSupplier, DoubleSupplier ySupplier,
            DoubleSupplier rotationSupplier, BooleanSupplier lockSupplier) {
        this.swerve = swerve;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.rotationSupplier = rotationSupplier;
        this.lockSupplier = lockSupplier;

        xLimiter = new SlewRateLimiter(Constants.Swerve.SWERVE_MAX_SPEED);
        yLimiter = new SlewRateLimiter(Constants.Swerve.SWERVE_MAX_SPEED);
        rotationLimiter = new SlewRateLimiter(Constants.Swerve.SWERVE_ROTATION_MAX_SPEED);
        addRequirements(swerve);
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
        swerve.setDrivebaseWheelVectors(xSupplier.getAsDouble(), ySupplier.getAsDouble(), rotationSupplier.getAsDouble(), true, false);
        executeLogging(xSupplier.getAsDouble(), ySupplier.getAsDouble(), rotationSupplier.getAsDouble());
    }

    protected void executeLogging(double xSpeed, double ySpeed, double rotationSpeed) {
        Logger.recordOutput("AutonSwerveDrive/xSpeed", xSpeed);
        Logger.recordOutput("AutonSwerveDrive/ySpeed", ySpeed);
        Logger.recordOutput("AutonSwerveDrive/inputX", xSupplier.getAsDouble());
        Logger.recordOutput("AutonSwerveDrive/inputY", ySupplier.getAsDouble());
    //    Logger.getInstance().recordOutput("SwerveDrive/inputR", rotationSupplier.getAsDouble());
        SmartDashboard.putNumber("rspeed", rotationSpeed);
        SmartDashboard.putNumber("rotation 2d", swerve.getRotation2d().getDegrees());
        Logger.recordOutput("AutonSwerveDrive/rotation", swerve.getRotation2d().getDegrees());
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