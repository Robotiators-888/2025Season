// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PhotonVision;

/**
 * The SUB_PhotonVision class manages the robot's two PhotonVision cameras for AprilTag-based localization.
 * It initializes the cameras and `PhotonPoseEstimator` objects, which are used to calculate the robot's
 * position on the field based on the AprilTags it can see. It provides methods to get the estimated
 * robot pose from each camera and access information about the best-seen target.
 * This class follows a singleton pattern.
 */
public class SUB_PhotonVision extends SubsystemBase {
  /** The singleton instance of the PhotonVision subsystem. */
  private static SUB_PhotonVision INSTANCE = null;

  /** The first PhotonVision camera, typically named in the PhotonVision web interface. */
  private final PhotonCamera cam1 = new PhotonCamera(PhotonVision.kCam1Name);

  /** The second PhotonVision camera. */
  private final PhotonCamera cam2 = new PhotonCamera(PhotonVision.kCam2Name);

  /** The most recently seen best target from the first camera. */
  private PhotonTrackedTarget cam1BestTarget;

  /** The most recently seen best target from the second camera. */
  private PhotonTrackedTarget cam2BestTarget;

  /** The pose estimator for the first camera, which calculates robot pose from AprilTags. */
  private final PhotonPoseEstimator poseEstimator1;

  /** The pose estimator for the second camera. */
  private final PhotonPoseEstimator poseEstimator2;

  /** The layout of the AprilTags on the competition field. */
  public AprilTagFieldLayout at_field;

  /**
   * Returns the singleton instance of the PhotonVision subsystem.
   * @return The singleton instance of SUB_PhotonVision.
   */
  public static SUB_PhotonVision getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new SUB_PhotonVision();
    }
    return INSTANCE;
  }

  /**
   * Constructs a new SUB_PhotonVision.
   * This constructor is private to enforce the singleton pattern.
   * It initializes the AprilTag field layout, cameras, and pose estimators with their respective configurations.
   */
  private SUB_PhotonVision() {
    // Load the AprilTag field layout for the current competition.
    // Note: This needs to be updated for different events with different field layouts.
    at_field =  AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

    cam1.setPipelineIndex(0);
    cam2.setPipelineIndex(0);

    poseEstimator1 = new PhotonPoseEstimator(at_field, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        PhotonVision.kRobotToCamera1);
    poseEstimator2 = new PhotonPoseEstimator(at_field, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
         PhotonVision.kRobotToCamera2);
    poseEstimator1.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    poseEstimator2.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
  }

  /**
   * Returns the estimated robot pose from the first camera, if a valid target is seen.
   * An EstimatedRobotPose contains the pose, timestamp, and the targets used for estimation.
   * @return An Optional containing the EstimatedRobotPose, or an empty Optional if no pose is available.
   */
  public Optional<EstimatedRobotPose> getCam1Pose() {
    List<PhotonPipelineResult> results1 = cam1.getAllUnreadResults();
  
    Optional<EstimatedRobotPose> finalPose1 = Optional.empty();
    for (PhotonPipelineResult result : results1) {
      if (result.hasTargets()) {
        cam1BestTarget = result.getBestTarget();
        finalPose1 = poseEstimator1.update(result);
      }
    }
    return finalPose1;
  }

  /**
   * Returns the estimated robot pose from the second camera, if a valid target is seen.
   * An EstimatedRobotPose contains the pose, timestamp, and the targets used for estimation.
   * @return An Optional containing the EstimatedRobotPose, or an empty Optional if no pose is available.
   */
  public Optional<EstimatedRobotPose> getCam2Pose() {
    List<PhotonPipelineResult> results2 = cam2.getAllUnreadResults();
    Optional<EstimatedRobotPose> finalPose2 = Optional.empty();
    for (PhotonPipelineResult result : results2) {
      if (result.hasTargets()) {
        cam2BestTarget = result.getBestTarget();
        finalPose2 = poseEstimator2.update(result);
      }
    }
    return finalPose2;
  }

  /**
   * Returns the best target seen by the first camera.
   * @return The best PhotonTrackedTarget from camera 1, or null if no target has been seen.
   */
  public PhotonTrackedTarget getCam1BestTarget() {
    return cam1BestTarget;
  }

  /**
   * Returns the best target seen by the second camera.
   * @return The best PhotonTrackedTarget from camera 2, or null if no target has been seen.
   */
  public PhotonTrackedTarget getCam2BestTarget() {
    return cam2BestTarget;
  }

  /**
   * Returns the yaw of a given tracked target.
   * @param target The target to get the yaw from.
   * @return The yaw of the target in degrees.
   */
  public double getTargetYaw(PhotonTrackedTarget target) {
    return target.getYaw();
  }

  /**
   * Returns the pitch of a given tracked target.
   * @param target The target to get the pitch from.
   * @return The pitch of the target in degrees.
   */
  public double getTargetPitch(PhotonTrackedTarget target) {
    return target.getPitch();
  }

  /**
   * Returns the area of a given tracked target.
   * @param target The target to get the area from.
   * @return The area of the target in screen percentage.
   */
  public double getTargetArea(PhotonTrackedTarget target) {
    return target.getArea();
  }

  /**
   * Returns the fiducial ID of a given tracked target.
   * @param target The target to get the ID from.
   * @return The fiducial ID of the target.
   */
  public int getId(PhotonTrackedTarget target) {
    return target.getFiducialId();
  }

  /**
   * This method is called periodically every robot loop.
   * It is currently empty but can be used for continuous tasks.
   */
  @Override
  public void periodic() {

  }
}
