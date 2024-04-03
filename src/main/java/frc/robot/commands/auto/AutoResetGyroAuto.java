// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import frc.robot.Constants;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class AutoResetGyroAuto extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Swerve swerve;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutoResetGyroAuto(Swerve subsystem) {
    swerve = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d startPose = new Pose2d();
    if (DriverStation.getAlliance().isPresent()) {
      DriverStation.Alliance robotAlliance = DriverStation.getAlliance().get();
      if (robotAlliance.equals(DriverStation.Alliance.Blue)) startPose = new Pose2d(new Translation2d(1.36, 5.55), swerve.getRotation2d());
      else if (robotAlliance.equals(DriverStation.Alliance.Red)) startPose = new Pose2d(new Translation2d(15.2, 5.55), swerve.getRotation2d());
    }
    swerve.resetOdometryPose(startPose);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
