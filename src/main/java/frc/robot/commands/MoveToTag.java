// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Swerve;
import frc.robot.Constants;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.PoseEstimator;

import java.util.concurrent.Future;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class MoveToTag extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private final Limelight limelight;
  private final PoseEstimator poseEstimator;
  private final Swerve swerve;

  private final java.util.logging.Logger logger;
  private HolonomicDriveController controller;

  private Future<Trajectory> futureTrajectory;
  private Trajectory trajectory;
  private boolean isTrajectoryGenerated;
  

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public MoveToTag(Limelight limelight, PoseEstimator poseEstimator, Swerve swerve) {
    this.limelight = limelight;
    this.poseEstimator = poseEstimator;
    this.swerve = swerve;

    logger = java.util.logging.Logger.getLogger(Swerve.class.getName());

    controller = new HolonomicDriveController(Constants.Scoring.X_CONTROLLER, Constants.Scoring.Y_CONTROLLER, Constants.Scoring.THETA_CONTROLLER);
    addRequirements(poseEstimator, limelight, swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
