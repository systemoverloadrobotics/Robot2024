// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.assistants.PoseEstimatorHelper;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Swerve;
import frc.sorutil.motor.SuController.ControlMode;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class MoveToSpeakerAngle extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final frc.robot.subsystems.Pivot pivot;
  private final Swerve swerve;
  private LinearFilter lf;
  private boolean flag;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public MoveToSpeakerAngle(frc.robot.subsystems.Pivot subsystem, Swerve s) {
    lf = LinearFilter.movingAverage(10);
    pivot = subsystem;
    swerve = s;
    flag = false;
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
    pivot.moveToSpeakerAngle(swerve);
    double calc = lf.calculate(pivot.pivot.outputPosition());
    
    if ((calc > PoseEstimatorHelper.angleShootEstimate - .5) && (calc < PoseEstimatorHelper.angleShootEstimate + .5)) {
      flag = true;
    }

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
