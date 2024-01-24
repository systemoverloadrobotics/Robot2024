// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class PoseEstimator extends SubsystemBase {

  private Swerve swerve;
  private Limelight limelight;

  private final java.util.logging.Logger logger;

  private Pose2d prevPose2D;
  private SwerveDrivePoseEstimator poseEstimator;

  public PoseEstimator(Swerve swerve, Limelight limelight) {
    logger = java.util.logging.Logger.getLogger(PoseEstimator.class.getName());
    this.swerve = swerve;
    this.limelight = limelight;

    //TODO: Figure out initial pose for the pose estimator below.
    poseEstimator = new SwerveDrivePoseEstimator(Constants.RobotDimensions.SWERVE_DRIVE_KINEMATICS,
      swerve.getRotation2d(),
      swerve.getModulePositions(), null);
    prevPose2D = poseEstimator.getEstimatedPosition();
  }



  @Override
  public void periodic() {
    Logger.recordOutput("LimeLight/DistanceToTarget(x)",table.getEntry("tx").getDouble(0.0));
    Logger.recordOutput("LimeLight/DistanceToTarget(y)",table.getEntry("ty").getDouble(0.0));
    Logger.recordOutput("LimeLight/TargetAreaVisable",table.getEntry("ta").getDouble(0.0));
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
