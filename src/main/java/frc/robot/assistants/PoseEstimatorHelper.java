// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.assistants;
import frc.robot.Constants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.Timer;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class PoseEstimatorHelper {
  public static Pose2d prevPose2D = new Pose2d();
  public static double lastTimestamp = 0;
  public static double angleShootEstimate = 0;
  public static InterpolatingDoubleTreeMap visionStdLookup = new InterpolatingDoubleTreeMap();
  // public static PhotonCamera cam = new PhotonCamera("Lemon");
  // public static PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(Constants.Vision.APRIL_TAG_FIELD_LAYOUT, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cam, Constants.Vision.ROBOT_TO_CAM);;

  static {
    visionStdLookup.put(0.1, 0.8);
    visionStdLookup.put(0.5, 1.0);
    visionStdLookup.put(1.0, 2.0);
    visionStdLookup.put(2.0, 2.5);
  }

  public static Pose2d getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    try {
      prevPose2D = prevEstimatedRobotPose;
      lastTimestamp = Timer.getFPGATimestamp() - (LimelightHelpers.getBotPose("limelight-lemon")[6] / 1000.0);
      System.out.println(lastTimestamp);
      System.out.println(LimelightHelpers.getBotPose2d_wpiBlue("limelight-lemon"));
      return LimelightHelpers.getBotPose2d_wpiBlue("limelight-lemon");
    } catch (Exception e) {
      e.printStackTrace();
      prevPose2D = prevEstimatedRobotPose;
      return prevPose2D;
    }
  }
}
