// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.Constants;
import edu.wpi.first.math.geometry.Pose2d;


import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class PoseEstimator {
  public static Pose2d prevPose2D;
  public static PhotonCamera cam = new PhotonCamera("Lemon");
  public static PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(Constants.Vision.APRIL_TAG_FIELD_LAYOUT, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cam, Constants.Vision.ROBOT_TO_CAM);;

  public static Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        Optional<EstimatedRobotPose> temp = photonPoseEstimator.update();
        prevPose2D = temp.map(x -> x.estimatedPose.toPose2d()).orElse(prevPose2D);
        return temp;
  }
}
