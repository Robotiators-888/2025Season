// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PhotonVision;

public class SUB_PhotonVision extends SubsystemBase {
  private static SUB_PhotonVision INSTANCE = null;

  private final PhotonCamera cam1 = new PhotonCamera(PhotonVision.kCam1Name);
  private final PhotonCamera cam2 = new PhotonCamera(PhotonVision.kCam2Name);
  private PhotonTrackedTarget bestTarget;
  private PhotonTrackedTarget bestTarget1;
  private PhotonTrackedTarget bestTarget2;
  private final PhotonPoseEstimator poseEstimator1;
  private final PhotonPoseEstimator poseEstimator2;
  public AprilTagFieldLayout at_field;

  public static SUB_PhotonVision getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new SUB_PhotonVision();
    }
    return INSTANCE;
  }

  private SUB_PhotonVision() {
    try {
      at_field = new AprilTagFieldLayout(
          Filesystem.getDeployDirectory().toPath().resolve("2025_reefscape_apriltags.json"));
      SmartDashboard.putBoolean("DEBUG/FILE FOUND?", true);
    } catch (IOException e) {
      SmartDashboard.putBoolean("DEBUG/FILE FOUND?", false);
    }

    poseEstimator1 = new PhotonPoseEstimator(at_field, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        PhotonVision.kRobotToCamera1);
    poseEstimator2 = new PhotonPoseEstimator(at_field, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        PhotonVision.kRobotToCamera2);
    poseEstimator1.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    poseEstimator2.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
    List<PhotonPipelineResult> results1 = cam1.getAllUnreadResults();
    Optional<EstimatedRobotPose> finalPose1 = Optional.empty();
    for (PhotonPipelineResult result : results1) {
      if (result.hasTargets()) {
        bestTarget1 = result.getBestTarget();
        finalPose1 = poseEstimator1.update(result);
      }
    }
    List<PhotonPipelineResult> results2 = cam2.getAllUnreadResults();
    Optional<EstimatedRobotPose> finalPose2 = Optional.empty();
    for (PhotonPipelineResult result : results2) {
      if (result.hasTargets()) {
        bestTarget2 = result.getBestTarget();
        finalPose2 = poseEstimator2.update(result);
      }
    }
    if (bestTarget1.getPoseAmbiguity() < bestTarget2.getPoseAmbiguity()) {
      bestTarget = bestTarget1;
      return finalPose1;
    } else {
      bestTarget = bestTarget2;
      return finalPose2;
    }
  }

  public PhotonTrackedTarget getBestTarget() {
    return bestTarget;
  }

  public double getTargetYaw(PhotonTrackedTarget target) {
    return target.getYaw();
  }

  public double getTargetPitch(PhotonTrackedTarget target) {
    return target.getPitch();
  }

  public double getTargetArea(PhotonTrackedTarget target) {
    return target.getArea();
  }

  public int getId(PhotonTrackedTarget target) {
    return target.getFiducialId();
  }

  @Override
  public void periodic() {
    var result = cam1.getLatestResult();
    if (result.hasTargets()) {
      bestTarget = result.getBestTarget();
    }
  }
}
