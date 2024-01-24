// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.sorutil.motor.MotorConfiguration;
import frc.sorutil.motor.SensorConfiguration;
import frc.sorutil.motor.SuController.ControlMode;
import frc.sorutil.motor.SuController.IdleMode;
import frc.sorutil.motor.SuSparkMax;

public class Pivot extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  SuSparkMax pivot;

  public Pivot() {
    MotorConfiguration pivotControllerConfig = new MotorConfiguration();
    SensorConfiguration pivotSensorConfig = new SensorConfiguration(new SensorConfiguration.IntegratedSensorSource(72));

    pivotControllerConfig.setPidProfile(Constants.Pivot.PID_PROFILE);
    pivotControllerConfig.setCurrentLimit(Constants.Pivot.PIVOT_CURRENT_LIMIT);
    pivotControllerConfig.setCurrentLimit(Constants.Pivot.PIVOT_MAX_OUTPUT);
    pivotControllerConfig.setIdleMode(IdleMode.BRAKE);

    pivot = new SuSparkMax(
      new CANSparkMax(Constants.Motor.CLIMB_LEFT, MotorType.kBrushless), "Climber Left", pivotControllerConfig, pivotSensorConfig
     );
  }

  // ADD FEED FORWARD

  public void stow() {
    pivot.set(ControlMode.POSITION, 90);
  }

  public void intake() {
    pivot.set(ControlMode.POSITION, 10);
  }

  public void stop() {
    pivot.set(ControlMode.PERCENT_OUTPUT, 0);
  }
  public double amperageMotors() {
    return ((CANSparkMax) pivot.rawController()).getOutputCurrent();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
