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
import frc.sorutil.motor.SuSparkMax;

public class Intake extends SubsystemBase {

  SuSparkMax intakeTop;
  SuSparkMax intakeBottom;
  SuSparkMax outtakeTop;
  SuSparkMax outtakeBottom;

 
  
  /** Creates a new ExampleSubsystem. */
  public Intake() {
    MotorConfiguration intakeControllerConfig = new MotorConfiguration();
    SensorConfiguration intakeSensorConfig = new SensorConfiguration(new SensorConfiguration.IntegratedSensorSource(3));

    MotorConfiguration outtakeControllerConfig = new MotorConfiguration();
    SensorConfiguration outtakeSensorConfig = new SensorConfiguration(new SensorConfiguration.IntegratedSensorSource(1));

    intakeControllerConfig.setCurrentLimit(Constants.Inouttake.INTAKE_CURRENT_LIMIT);
    intakeControllerConfig.setMaxOutput(Constants.Inouttake.INTAKE_MAX_OUTPUT);

    outtakeControllerConfig.setPidProfile(Constants.Inouttake.PID_PROFILE);
    outtakeControllerConfig.setCurrentLimit(Constants.Inouttake.OUTTAKE_CURRENT_LIMIT);
    outtakeControllerConfig.setMaxOutput(Constants.Inouttake.OUTTAKE_MAX_OUTPUT);

    intakeTop = new SuSparkMax(
      new CANSparkMax(Constants.Motor.ROLLER_INTAKE_TOP, MotorType.kBrushless), "Top Intake", intakeControllerConfig, intakeSensorConfig
    );
    intakeBottom = new SuSparkMax(
      new CANSparkMax(Constants.Motor.ROLLER_INTAKE_BOTTOM, MotorType.kBrushless), "Bottom Intake", intakeControllerConfig, intakeSensorConfig
    );
    outtakeTop = new SuSparkMax(
      new CANSparkMax(Constants.Motor.ROLLER_OUTTAKE_TOP, MotorType.kBrushless), "Top Outtake", outtakeControllerConfig, outtakeSensorConfig
    );
    outtakeBottom = new SuSparkMax(
      new CANSparkMax(Constants.Motor.ROLLER_OUTTAKE_BOTTOM, MotorType.kBrushless), "Bottom Outtake", outtakeControllerConfig, outtakeSensorConfig
    );    


  }
  public void intake() {
    //subject to change
    intakeTop.set(ControlMode.PERCENT_OUTPUT, 0.2);
    intakeBottom.set(ControlMode.PERCENT_OUTPUT, -0.2);
  }
  public void retraction() {
    intakeTop.set(ControlMode.PERCENT_OUTPUT, -0.04);
  }

  public void setFlywheels() {
    outtakeTop.set(ControlMode.VELOCITY, 3000);
    outtakeBottom.set(ControlMode.VELOCITY, 3000);
    //holds note in place
    intakeTop.set(ControlMode.PERCENT_OUTPUT, 0);
    intakeBottom.set(ControlMode.PERCENT_OUTPUT, 0);
  }

  public void outtake() {
    intakeTop.set(ControlMode.PERCENT_OUTPUT, 0.2);
    intakeBottom.set(ControlMode.PERCENT_OUTPUT, -0.2);
    outtakeTop.set(ControlMode.VELOCITY, 3000);
    outtakeBottom.set(ControlMode.VELOCITY, 3000);
  }

  public double amperageMotorsIntake() {
    return Math.max(((CANSparkMax) intakeTop.rawController()).getOutputCurrent(), ((CANSparkMax) intakeBottom.rawController()).getOutputCurrent());
  }
  
  public double amperageMotorsOuttake() {
    return Math.max(((CANSparkMax) outtakeTop.rawController()).getOutputCurrent(), ((CANSparkMax) outtakeBottom.rawController()).getOutputCurrent());
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