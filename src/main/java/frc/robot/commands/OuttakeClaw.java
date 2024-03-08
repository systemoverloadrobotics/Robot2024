// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;
import frc.sorutil.motor.SuController.ControlMode;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class OuttakeClaw extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Intake intake;
  private final Pivot hitarth;
  private final LinearFilter lf;
  
  public OuttakeClaw(Intake subsystem, Pivot hitarth) {
    intake = subsystem;
    this.hitarth = hitarth;
    this.lf = LinearFilter.movingAverage(10);
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Logger.recordOutput("UTBAE", intake.outtakeBottom.outputVelocity());
    double value = intake.outtakeBottom.outputVelocity();
    intake.setFlywheels();
    if (value >= 4000) { // TODO: Change Values to Constants
        intake.outtake();
    }
    if (value >= 500 && hitarth.angle > 90) { // TODO: Change Values to Constants
        intake.outtake();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
