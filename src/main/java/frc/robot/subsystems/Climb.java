// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.sorutil.motor.MotorConfiguration;
import frc.sorutil.motor.SensorConfiguration;
import frc.sorutil.motor.SensorConfiguration.ConnectedSensorType;
import frc.sorutil.motor.SuController.ControlMode;
import frc.sorutil.motor.SuController.IdleMode;
import frc.sorutil.motor.SuSparkMax;

public class Climb extends SubsystemBase {
  SuSparkMax climberLeft;
  SuSparkMax climberRight;

  public Climb() {
    MotorConfiguration climberControllerConfig = new MotorConfiguration();
    SensorConfiguration climberSensorConfig = new SensorConfiguration(new SensorConfiguration.IntegratedSensorSource(16));

    climberControllerConfig.setPidProfile(Constants.Climb.PID_PROFILE);
    climberControllerConfig.setCurrentLimit(Constants.Climb.CLIMB_CURRENT_LIMIT);
    climberControllerConfig.setMaxOutput(Constants.Climb.CLIMB_MAX_OUTPUT);
    
    climberControllerConfig.setIdleMode(IdleMode.BRAKE); // set to brake mode
    
    climberLeft = new SuSparkMax(
      new CANSparkMax(Constants.Motor.ACTUAL_CLIMB_LEFT, MotorType.kBrushless), "Climber Left", climberControllerConfig, climberSensorConfig
    );
    climberRight = new SuSparkMax(
      new CANSparkMax(Constants.Motor.ACTUAL_CLIMB_RIGHT, MotorType.kBrushless), "Climber Right", climberControllerConfig, climberSensorConfig
    );
  }

  public void up() {
    climberLeft.set(ControlMode.PERCENT_OUTPUT, 0.8);
    climberRight.set(ControlMode.PERCENT_OUTPUT, 0.8);
  }

  public void down() {
    climberLeft.set(ControlMode.PERCENT_OUTPUT, -0.8);
    climberRight.set(ControlMode.PERCENT_OUTPUT, -0.8);
  }

  public void stop() {
    climberLeft.set(ControlMode.PERCENT_OUTPUT, 0);
    climberRight.set(ControlMode.PERCENT_OUTPUT, 0);
  }

  public double amperageMotors() {
    return Math.max(((CANSparkMax) climberLeft.rawController()).getOutputCurrent(), ((CANSparkMax) climberRight.rawController()).getOutputCurrent());
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
