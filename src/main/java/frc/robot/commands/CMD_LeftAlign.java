// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Operator;
import frc.robot.subsystems.SUB_Drivetrain;
import frc.robot.subsystems.SUB_PhotonVision;

public class CMD_LeftAlign extends Command {
  SUB_PhotonVision photonVision;
  SUB_Drivetrain drivetrain;

  Pose2d alignedTagPose;
  Pose2d targetPose;
  Integer targetId;

  CommandXboxController driverController;

  private final PIDController xController = new PIDController(0.5, 0.01, 0);
  private final PIDController yController = new PIDController(0.5, 0.01, 0);
  private final PIDController robotAngleController = new PIDController(0.5, 0.01, 0); // 0.25, 0, 0

  /** Creates a new CMD_AdjustPivotOnDist. */
  public CMD_LeftAlign(SUB_Drivetrain drivetrain, SUB_PhotonVision photonVision) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.photonVision = photonVision;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    var alliance = DriverStation.getAlliance();

    int[] targetTagSet;

    if (alliance.isPresent()) {
      if (alliance.get() == DriverStation.Alliance.Red) {

        targetTagSet={6,7,8,9,10,11};

      } else {
        targetTagSet={17,18,19,20,21,22};
      }
    } else {
      SmartDashboard.putBoolean("Alliance Error", true);
      end(true);
    }


    double minDistance = Double.MAX_VALUE;
    for (int tag : targetTagSet) {
      Pose2d tagPose = drivetrain.at_field.getTagPose(tag).get().toPose2d();
      Translation2d translate = tagPose.minus(drivetrain.getPose()).getTranslation();
      double distance = Math.sqrt(Math.pow(translate.getX(), 2) + Math.pow(translate.getY(), 2));

      if (distance < minDistance) {
        alignedTagPose = tagPose;
        targetId = tag;
      }
    }

    robotAngleController.setTolerance(0.07);
    SmartDashboard.putNumber("rot radians", alignedTagPose.getRotation().getAngle());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d currentPose = drivetrain.getPose();
    double target = alignedTagPose.getRotation().getAngle() + Math.PI;

    if (target > Math.PI) {
      target = -Math.PI + alignedTagPose.getRotation().getAngle();
    }

    drivetrain.drive(
        -MathUtil.applyDeadband(Math.copySign(Math.pow(driverController.getRawAxis(1), 2),
            driverController.getRawAxis(1)), Operator.kDriveDeadband),
        -MathUtil.applyDeadband(Math.copySign(Math.pow(driverController.getRawAxis(0), 2),
            driverController.getRawAxis(0)), Operator.kDriveDeadband),

        robotAngleController.calculate(currentPose.getRotation().getRadians(), target), false,
        true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return 
        && robotAngleController.atSetpoint();
  }
}
