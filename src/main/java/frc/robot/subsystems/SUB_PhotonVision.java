// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.EstimatedRobotPose;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PhotonVision;

public class SUB_PhotonVision extends SubsystemBase {
  public static SUB_PhotonVision INSTANCE = null;

  private PhotonCamera cam1 = new PhotonCamera(PhotonVision.kCam1Name);


  private PhotonTrackedTarget bestTarget;
  public PhotonPoseEstimator poseEstimator;
  public boolean hasResults = false;

  public AprilTagFieldLayout at_field;

  public static SUB_PhotonVision getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new SUB_PhotonVision();
    }
    return INSTANCE;
  }

  /** Creates a new SUB_PhotonVision. */
  private SUB_PhotonVision() {
    try {
      at_field = new AprilTagFieldLayout(
          Filesystem.getDeployDirectory().toPath().resolve("2025_reefscape_apriltags.json"));
      SmartDashboard.putBoolean("DEBUG/FILE FOUND?", true);
    } catch (IOException e) {
      SmartDashboard.putBoolean("DEBUG/FILE FOUND?", false);
    }

    poseEstimator = new PhotonPoseEstimator(at_field, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        PhotonVision.kRobotToCamera1);
    poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
  }


  // Called periodically by robot container
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
    List<PhotonPipelineResult> results = cam1.getAllUnreadResults(); // Gets queue of results
                                                                     // (earliest first??)
    Optional<EstimatedRobotPose> finalPose = Optional.empty();
    for (PhotonPipelineResult result : results) {
      if (result.hasTargets()) {
        bestTarget = result.getBestTarget();
        finalPose = poseEstimator.update(result);
      }
    }
    return finalPose;
  }

  // public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
  //   var result = cam1.getLatestResult();
  //   if (result.hasTargets()) {
  //       return poseEstimator.update(result);
  //   }
  //   return Optional.empty();
  // }

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
