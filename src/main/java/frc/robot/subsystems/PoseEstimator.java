// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;

import org.littletonrobotics.junction.Logger;

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
    poseEstimator = new SwerveDrivePoseEstimator(
      Constants.RobotDimensions.SWERVE_DRIVE_KINEMATICS,
      swerve.getRotation2d(),
      swerve.getModulePositions(),
      new Pose2d(),Constants.PoseEstimation.POSE_GYRO_STD, Constants.PoseEstimation.POSE_VISION_STD);
    prevPose2D = poseEstimator.getEstimatedPosition();
  }

  public Pose2d getEstimatPose2d() {
      return poseEstimator.getEstimatedPosition();
  }
  
  @Override
  public void periodic() {
    Logger.recordOutput("LimeLight/DistanceToTarget(x)", LimelightHelpers.getTX());
    Logger.recordOutput("LimeLight/DistanceToTarget(y)", LimelightHelpers.getTY());
    Logger.recordOutput("LimeLight/TargetAreaVisable", LimelightHelpers.getTA());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
