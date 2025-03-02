// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;
import java.util.Arrays;
import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants;
import frc.robot.subsystems.SUB_Drivetrain;
import frc.robot.subsystems.SUB_PhotonVision;

public class CMD_ReefAlign extends RunCommand {
  private final SUB_PhotonVision photonVision;
  private final SUB_Drivetrain drivetrain;

  private Pose2d tagPose;
  private Integer targetId;

  private final boolean isLeftAlign;
  private List<Integer> targetTagSet;

  private final PIDController xController = new PIDController(0.2, 0, 0.02);
  private final PIDController yController = new PIDController(0.2, 0, 0.02);
  private final PIDController robotAngleController = new PIDController(0.7, 0, 0);

  private final double xMagnitude = Constants.Drivetrain.kXShiftMagnitude;
  private final double yMagnitude = Constants.Drivetrain.kYShiftMagnitude;

  public CMD_ReefAlign(SUB_Drivetrain drivetrain, SUB_PhotonVision photonVision, boolean isLeftAlign) {
    super(() -> {}, drivetrain);
    this.drivetrain = drivetrain;
    this.photonVision = photonVision;
    this.isLeftAlign = isLeftAlign;
    robotAngleController.enableContinuousInput(-Math.PI, Math.PI);
    robotAngleController.setTolerance(Units.degreesToRadians(1));
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    xController.setTolerance(0);
    yController.setTolerance(0);
    robotAngleController.setTolerance(0);

    Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      targetTagSet = alliance.get() == DriverStation.Alliance.Red
          ? Arrays.asList(7, 8, 9, 10, 11, 6)
          : Arrays.asList(21, 20, 19, 18, 17, 22);
    } else {
      SmartDashboard.putBoolean("Alliance Error", true);
      end(true);
      return;
    }

    double minDistance = Double.MAX_VALUE;
    for (int tag : targetTagSet) {
      Pose2d pose = photonVision.at_field.getTagPose(tag).orElse(new Pose3d()).toPose2d();
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
    double angle = Units.degreesToRadians(60 * targetTagSet.indexOf(targetId));
    double offset = Units.degreesToRadians(isLeftAlign ? 90 : -90);

    double x = xMagnitude * Math.cos(angle) + yMagnitude * Math.cos(angle + offset);
    double y = xMagnitude * Math.sin(angle) + yMagnitude * Math.sin(angle + offset);

    drivetrain.publisher1.set(new Pose2d(tagPose.getX() + x, tagPose.getY() + y, tagPose.getRotation()));

    double xSpeed = xController.calculate(currentPose.getX(), tagPose.getX() + x);
    double ySpeed = yController.calculate(currentPose.getY(), tagPose.getY() + y);
    double omegaSpeed = robotAngleController.calculate(MathUtil.angleModulus(currentPose.getRotation().getRadians()), MathUtil.angleModulus(tagPose.getRotation().getRadians()));

    drivetrain.drive(xSpeed, ySpeed, -omegaSpeed, true, false);
  }

  @Override
  public void end(boolean interrupted) {
    // No specific actions on end
  }

  @Override
  public boolean isFinished() {
    boolean atSetpoint = xController.atSetpoint() && yController.atSetpoint() && robotAngleController.atSetpoint();
    SmartDashboard.putBoolean("AlignCommandComplete!", atSetpoint);
    return atSetpoint;
  }
}
