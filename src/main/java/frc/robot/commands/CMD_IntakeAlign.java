// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants;
import frc.robot.subsystems.SUB_Drivetrain;
import frc.robot.subsystems.SUB_PhotonVision;

/**
 * The CMD_IntakeAlign class is a command that aligns the robot to the human player station
 * for intaking game pieces. It uses PID controllers to control the robot's x, y, and
 * rotational movement to align with a target AprilTag at the intake station.
 */
public class CMD_IntakeAlign extends RunCommand {
  /** The PhotonVision subsystem instance for accessing camera data. */
  private final SUB_PhotonVision photonVision;

  /** The drivetrain subsystem instance for controlling robot movement. */
  private final SUB_Drivetrain drivetrain;

  /** The pose of the target AprilTag on the field. */
  private Pose2d tagPose;

  /** The ID of the target AprilTag that the robot is aligning to. */
  private Integer targetId;

  /** A list of possible target AprilTag IDs for the current alliance (Red or Blue). */
  private List<Integer> targetTagSet;

  /** The PID controller for managing movement in the x-direction (forward/backward). */
  private final PIDController xController = new PIDController(0.1, 0, 0.02);

  /** The PID controller for managing movement in the y-direction (strafe left/right). */
  private final PIDController yController = new PIDController(0.5, 0, 0.05);

  /** The PID controller for managing the robot's rotation. */
  private final PIDController robotAngleController = new PIDController(0.5, 0, 0);

  /** A map of hardcoded target positions for each intake station AprilTag. */
  private HashMap<Integer, Pose2d> targetPositions = new HashMap<>();

  /**
   * Creates a new CMD_IntakeAlign command.
   * @param drivetrain The drivetrain subsystem to use for movement.
   * @param photonVision The PhotonVision subsystem to use for AprilTag detection.
   */
  public CMD_IntakeAlign(SUB_Drivetrain drivetrain, SUB_PhotonVision photonVision) {
    super(() -> {
    }, drivetrain);

    this.drivetrain = drivetrain;
    this.photonVision = photonVision;
    robotAngleController.enableContinuousInput(-Math.PI, Math.PI);
    robotAngleController.setTolerance(Units.degreesToRadians(1));

    // Populate the target positions map with hardcoded poses for each intake station AprilTag.
    // These are the desired final positions for the robot.
    targetPositions.put(13, new Pose2d(0.646, 6.602, photonVision.at_field.getTagPose(13).get().toPose2d().getRotation()));
    targetPositions.put(12, new Pose2d(0.646, 1.388, photonVision.at_field.getTagPose(12).get().toPose2d().getRotation()));

    targetPositions.put(2, new Pose2d(16.889, 6.602, photonVision.at_field.getTagPose(2).get().toPose2d().getRotation()));
    targetPositions.put(1, new Pose2d(16.889, 1.388, photonVision.at_field.getTagPose(1).get().toPose2d().getRotation()));

    addRequirements(drivetrain);
  }

  /**
   * Called when the command is initially scheduled.
   * This method determines the closest intake station AprilTag based on the robot's current
   * position and alliance color, then initializes the PID controllers for alignment.
   */
  @Override
  public void initialize() {
    xController.setTolerance(0);
    yController.setTolerance(0);
    robotAngleController.setTolerance(0);

    // Determine the set of target tags based on the current alliance color.
    Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      targetTagSet =
          alliance.get() == DriverStation.Alliance.Red ? Arrays.asList(1,2)
              : Arrays.asList(12, 13);
    } else {
      SmartDashboard.putBoolean("Alliance Error", true);
      end(true);
      return;
    }

    // Find the closest target tag to the robot.
    double minDistance = Double.MAX_VALUE;
    for (int tag : targetTagSet) {
      Pose2d pose = photonVision.at_field.getTagPose(tag).get().toPose2d();
      Translation2d translate = pose.minus(drivetrain.getPose()).getTranslation();
      double distance = translate.getNorm();

      if (distance < minDistance) {
        tagPose = pose;
        targetId = tag;
        minDistance = distance;
      }
    }
    robotAngleController.reset();
    xController.reset();
    yController.reset();
  }

  /**
   * Called every time the scheduler runs while the command is scheduled.
   * This method calculates the required speeds using the PID controllers to align with the
   * target pose and then drives the robot.
   */
  @Override
  public void execute() {
    Pose2d currentPose = drivetrain.getPose();
    Pose2d targetPose = targetPositions.get(targetId);

    drivetrain.publisher1.set(targetPose);

    // Calculate the speeds required to reach the target pose.
    double xSpeed = xController.calculate(currentPose.getX(), targetPose.getX());
    double ySpeed = yController.calculate(currentPose.getY(), targetPose.getY());
    double omegaSpeed = robotAngleController.calculate(
        MathUtil.angleModulus(currentPose.getRotation().getRadians()),
        MathUtil.angleModulus(targetPose.getRotation().getRadians()));

    drivetrain.drive(xSpeed, ySpeed, omegaSpeed, true, false);
    SmartDashboard.putNumber("X Error", currentPose.getX() - targetPose.getX());
    SmartDashboard.putNumber("Y Error", currentPose.getY() - targetPose.getY());
  }

  /**
   * Called once the command ends or is interrupted.
   * @param interrupted True if the command was interrupted, false otherwise.
   */
  @Override
  public void end(boolean interrupted) {
    // No specific actions on end, the drivetrain will stop via its default command.
  }

  /**
   * Returns true when the command should end.
   * @return True when the robot is at the target setpoint, false otherwise.
   */
  @Override
  public boolean isFinished() {
    boolean atSetpoint =
        xController.atSetpoint() && yController.atSetpoint() && robotAngleController.atSetpoint();
    SmartDashboard.putBoolean("AlignCommandComplete!", atSetpoint);
    return atSetpoint;
  }
}
