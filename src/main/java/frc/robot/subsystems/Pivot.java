// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.assistants.LimelightHelpers;
import frc.robot.assistants.PoseEstimatorHelper;
import frc.sorutil.SorMath;
import frc.sorutil.motor.MotorConfiguration;
import frc.sorutil.motor.SensorConfiguration;
import frc.sorutil.motor.SuController.ControlMode;
import frc.sorutil.motor.SuController.IdleMode;
import frc.sorutil.motor.SuSparkMax;

public class Pivot extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public SuSparkMax pivot;
  public SuSparkMax pivotB;
  public DutyCycleEncoder e;
  public double angle; // degrees

  private final TrapezoidProfile.Constraints constraintsAngle = new TrapezoidProfile.Constraints(360, 720);

  private TrapezoidProfile.State goalAngle;
  private TrapezoidProfile.State currentAngle;
  private TrapezoidProfile.State angleSetpoint = new TrapezoidProfile.State();

  public Pivot() {
    SmartDashboard.putNumber("thevelocitything idk", 0);
    e = new DutyCycleEncoder(0);
    MotorConfiguration pivotControllerConfig = new MotorConfiguration();
    SensorConfiguration pivotSensorConfig = new SensorConfiguration(new SensorConfiguration.IntegratedSensorSource(216));

    pivotControllerConfig.setPidProfile(Constants.Pivot.PID_PROFILE);
    pivotControllerConfig.setCurrentLimit(Constants.Pivot.PIVOT_CURRENT_LIMIT);
    pivotControllerConfig.setMaxOutput(Constants.Pivot.PIVOT_MAX_OUTPUT);
    pivotControllerConfig.setIdleMode(IdleMode.BRAKE);
    pivotControllerConfig.setInverted(true);

    pivot = new SuSparkMax(
      new CANSparkMax(Constants.Motor.CLIMB_LEFT, MotorType.kBrushless), "Climber Left", pivotControllerConfig, pivotSensorConfig
    );
    pivotControllerConfig.setInverted(false);
    pivotB = new SuSparkMax(
      new CANSparkMax(Constants.Motor.CLIMB_RIGHT, MotorType.kBrushless), "Climber Right", pivotControllerConfig, pivotSensorConfig
    );

    ((CANSparkMax) pivot.rawController()).setClosedLoopRampRate(2);
    ((CANSparkMax) pivotB.rawController()).setClosedLoopRampRate(2);

    resetPivot();
  }

  public void resetPivot() {
    angleSetpoint = new TrapezoidProfile.State(pivot.outputPosition(), 0);
    goalAngle = new TrapezoidProfile.State(pivot.outputPosition(), 0);
    currentAngle = new TrapezoidProfile.State(pivot.outputPosition(), 0);

    e.setPositionOffset(0);
    pivot.setSensorPosition(0);
  }

  // TODO: ADD FEED FORWARD

  public void stow() {
    pivot.set(ControlMode.POSITION, 90);
  }

  public void moveToIntakeAngle() {
    goalAngle = new TrapezoidProfile.State(Constants.Pivot.INTAKE_ANGLE, 0);
  }
  public void moveToAmpAngle() {
    goalAngle = new TrapezoidProfile.State(Constants.Pivot.AMP_ANGLE, 0);
  }
  public void moveToSpeakerAngle(Swerve swerve) {
    goalAngle = new TrapezoidProfile.State(PoseEstimatorHelper.angleShootEstimate, 0);
  }
  public void moveToStowAngle() {
    goalAngle = new TrapezoidProfile.State(Constants.Pivot.STOW_ANGLE, 0);
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
    Logger.recordOutput("Pivot/AngleRaw", e.get());
    Logger.recordOutput("Pivot/Angle", 80 * e.get());

    // goalAngle = new TrapezoidProfile.State(SmartDashboard.getNumber("thevelocitything idk", 0), 0);
    TrapezoidProfile profile = new TrapezoidProfile(constraintsAngle, goalAngle, angleSetpoint);
    angleSetpoint = profile.calculate(Constants.ROBOT_PERIOD, currentAngle, goalAngle);
    Logger.recordOutput("tg", goalAngle.position);
    // SmartDashboard.getNumber("thevelocitything idk", 0) | goalAngle.position
    double o = goalAngle.position;
    angle = o;
    if (Math.abs(pivot.outputPosition() - o) < 0) {
      pivot.stop();
      pivotB.stop();
    }
    else {
      pivot.set(ControlMode.POSITION, o, 0.54 * Math.cos(Math.toRadians(80 * e.get())));
      pivotB.set(ControlMode.POSITION, o, 0.54 * Math.cos(Math.toRadians(80 * e.get())));
    }
  }

  // x in ft
  public static double getAngleNeeded(double x) {
    return 7.649 * Math.pow(x, 0.655) + 12.178;
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
