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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.SUB_Drivetrain;
import frc.robot.subsystems.SUB_PhotonVision;

public class CMD_IntakeAlign extends RunCommand {
  private final SUB_PhotonVision photonVision;
  private final SUB_Drivetrain drivetrain;

  private Pose2d tagPose;
  private Integer targetId;
  private List<Integer> targetTagSet;

  private final PIDController xController = new PIDController(0.1, 0, 0.02);
  private final PIDController yController = new PIDController(0.5, 0, 0.05);
  private final PIDController robotAngleController = new PIDController(0.5, 0, 0);
  private HashMap<Integer, Pose2d> targetPositions = new HashMap<>();

  public CMD_IntakeAlign(SUB_Drivetrain drivetrain, SUB_PhotonVision photonVision) {
    super(() -> {
    }, drivetrain);

    this.drivetrain = drivetrain;
    this.photonVision = photonVision;
    robotAngleController.enableContinuousInput(-Math.PI, Math.PI);
    robotAngleController.setTolerance(Units.degreesToRadians(1));

    targetPositions.put(13, new Pose2d(0.646, 6.602, photonVision.at_field.getTagPose(13).get().toPose2d().getRotation()));
    targetPositions.put(12, new Pose2d(0.646, 1.388, photonVision.at_field.getTagPose(12).get().toPose2d().getRotation()));

    targetPositions.put(2, new Pose2d(16.889, 6.602, photonVision.at_field.getTagPose(2).get().toPose2d().getRotation()));
    targetPositions.put(1, new Pose2d(16.889, 1.388, photonVision.at_field.getTagPose(1).get().toPose2d().getRotation()));

    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    xController.setTolerance(0);
    yController.setTolerance(0);
    robotAngleController.setTolerance(0);

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

  @Override
  public void execute() {
    Pose2d currentPose = drivetrain.getPose();
    Pose2d targetPose = targetPositions.get(targetId);

    drivetrain.publisher1.set(targetPose);

    double xSpeed = xController.calculate(currentPose.getX(), targetPose.getX());
    double ySpeed = yController.calculate(currentPose.getY(), targetPose.getY());
    double omegaSpeed = robotAngleController.calculate(
        MathUtil.angleModulus(currentPose.getRotation().getRadians()),
        MathUtil.angleModulus(targetPose.getRotation().getRadians()));

    drivetrain.drive(xSpeed, ySpeed, omegaSpeed, true, false);
    SmartDashboard.putNumber("X Error", currentPose.getX() - targetPose.getX());
    SmartDashboard.putNumber("Y Error", currentPose.getY() - targetPose.getY());
  }

  @Override
  public void end(boolean interrupted) {
    // No specific actions on end
  }

  @Override
  public boolean isFinished() {
    boolean atSetpoint =
        xController.atSetpoint() && yController.atSetpoint() && robotAngleController.atSetpoint();
    SmartDashboard.putBoolean("AlignCommandComplete!", atSetpoint);
    return atSetpoint;
  }
}
