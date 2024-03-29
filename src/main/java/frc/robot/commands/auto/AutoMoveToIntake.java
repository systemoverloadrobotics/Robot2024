// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import frc.robot.Constants;
import frc.robot.subsystems.Pivot;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class AutoMoveToIntake extends Command {


  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Pivot pivot;

  private LinearFilter lf;
  /**
   * Creates a new ExampleCommand.
   *
   * @param pivot The subsystem used by this command.
   */
  public AutoMoveToIntake(Pivot pivot) {
    lf = LinearFilter.movingAverage(20);
    this.pivot = pivot;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(pivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pivot.moveToIntakeAngle();
    double calc = lf.calculate(pivot.pivot.outputPosition());
    
    if ((calc > Constants.Pivot.INTAKE_ANGLE - .5) && (calc < Constants.Pivot.INTAKE_ANGLE + .5)) {
      end(true);
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
