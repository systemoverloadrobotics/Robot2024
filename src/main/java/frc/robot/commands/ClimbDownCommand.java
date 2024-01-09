// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Climb;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.Command;

public class ClimbDownCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Climb climb;
  private final LinearFilter lf;

  public ClimbDownCommand(Climb climb) {
    this.climb = climb;
    this.lf = LinearFilter.movingAverage(10);

    addRequirements(climb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climb.down();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (lf.calculate(climb.amperageMotors()) > 10) {
      climb.stop();
      end(true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
