// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PhotonVision;
import frc.robot.subsystems.SUB_Drivetrain;
import frc.robot.subsystems.SUB_PhotonVision;

public class CMD_DriveToClosestGroundObject extends Command {
  private final SUB_Drivetrain drivetrain;
  private final SUB_PhotonVision photonVision;
  private final PhotonCamera cam1;
  private final PhotonCamera cam2;

  // PID controllers for driving to the Coral
  private final PIDController xController = new PIDController(1.0, 0, 0);
  private final PIDController yController = new PIDController(1.0, 0, 0);
  private final PIDController rotController = new PIDController(1.0, 0, 0);

  private PhotonTrackedTarget closestCoral;
  private double lastCoralArea = 0;
  private double lastCoralYaw = 0;
  private double lastCoralPitch = 0;
  private boolean CoralFound = false;

  /** Creates a new CMD_DriveToClosestGroundObject. */
  public CMD_DriveToClosestGroundObject(SUB_Drivetrain drivetrain, SUB_PhotonVision photonVision) {
    this.drivetrain = drivetrain;
    this.photonVision = photonVision;

    // Get the cameras from PhotonVision
    this.cam1 = new PhotonCamera(PhotonVision.kCam1Name);
    this.cam2 = new PhotonCamera(PhotonVision.kCam2Name);

    // Set the pipeline index to the ground object detection pipeline
    // Note: You need to create this pipeline in PhotonVision dashboard
    // Pipeline 0 is typically AprilTags, so we'll use pipeline 1 for ground objects
    cam1.setPipelineIndex(1);
    cam2.setPipelineIndex(1);

    // Configure PID controllers
    rotController.enableContinuousInput(-Math.PI, Math.PI);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Reset PID controllers
    xController.reset();
    yController.reset();
    rotController.reset();

    // Set pipeline to ground object detection
    cam1.setPipelineIndex(1);
    cam2.setPipelineIndex(1);

    CoralFound = false;
    closestCoral = null;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Get the latest results from both cameras
    List<PhotonPipelineResult> results1 = cam1.getAllUnreadResults();
    List<PhotonPipelineResult> results2 = cam2.getAllUnreadResults();

    // Process the results to find the best target from each camera
    PhotonPipelineResult result1 = results1.isEmpty() ? new PhotonPipelineResult() : results1.get(results1.size() - 1);
    PhotonPipelineResult result2 = results2.isEmpty() ? new PhotonPipelineResult() : results2.get(results2.size() - 1);

    // Check if either camera has detected targets
    if (result1.hasTargets() || result2.hasTargets()) {
      CoralFound = true;

      // Find the closest target (largest area) from either camera
      PhotonTrackedTarget bestCoral1 = result1.hasTargets() ? result1.getBestTarget() : null;
      PhotonTrackedTarget bestCoral2 = result2.hasTargets() ? result2.getBestTarget() : null;

      double area1 = bestCoral1 != null ? bestCoral1.getArea() : 0;
      double area2 = bestCoral2 != null ? bestCoral2.getArea() : 0;

      // Select the target with the largest area (closest)
      closestCoral = (area1 > area2) ? bestCoral1 : bestCoral2;

      if (closestCoral != null) {
        // Store target information
        lastCoralArea = closestCoral.getArea();
        lastCoralYaw = closestCoral.getYaw();
        lastCoralPitch = closestCoral.getPitch();

        // Convert yaw to radians for rotation
        double yawRadians = Math.toRadians(lastCoralYaw);

        // Calculate drive speeds based on target position
        // Yaw controls rotation to center the target
        double rotSpeed = rotController.calculate(0, yawRadians);

        // Forward speed based on target area (distance)
        // Slow down as we get closer (larger area)
        double forwardSpeed = 1.0 - (lastCoralArea / 100.0); // Adjust divisor based on max expected area
        forwardSpeed = Math.max(0.2, Math.min(forwardSpeed, 0.8)); // Limit speed range

        // Lateral speed to center the target horizontally
        double lateralSpeed = -yController.calculate(0, yawRadians);

        // Create chassis speeds for driving
        ChassisSpeeds speeds = new ChassisSpeeds(forwardSpeed, lateralSpeed, rotSpeed);

        // Drive the robot using the calculated speeds
        drivetrain.driveFieldRelative(speeds);

        // Output debug information
        SmartDashboard.putNumber("Target Yaw", lastCoralYaw);
        SmartDashboard.putNumber("Target Area", lastCoralArea);
        SmartDashboard.putNumber("Forward Speed", forwardSpeed);
        SmartDashboard.putNumber("Lateral Speed", lateralSpeed);
        SmartDashboard.putNumber("Rotation Speed", rotSpeed);
      }
    } else {
      // No target found, stop the robot
      CoralFound = false;
      drivetrain.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stop the robot
    drivetrain.stop();

    // Reset cameras to AprilTag pipeline
    cam1.setPipelineIndex(0);
    cam2.setPipelineIndex(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // End the command when we're very close to the target (large area)
    // or if we lose the target for too long
    if (CoralFound && lastCoralArea > 80) { // Adjust threshold based on testing
      return true;
    }
    return false;
  }
}