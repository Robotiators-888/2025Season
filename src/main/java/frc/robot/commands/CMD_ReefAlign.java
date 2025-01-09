// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.SUB_Drivetrain;
import frc.robot.subsystems.SUB_PhotonVision;

public class CMD_ReefAlign extends Command {
  SUB_PhotonVision photonVision;
  SUB_Drivetrain drivetrain;

  Pose2d tagPose;
  Integer targetId;

  boolean isLeftAlign;

  CommandXboxController driverController;

  private final PIDController xController = new PIDController(0.025, 0, 0); // 3, 0, 0
  private final PIDController yController = new PIDController(0.025, 0, 0);
  private final PIDController robotAngleController = new PIDController(0.05, 0.01, 0);

  /** Creates a new CMD_AdjustPivotOnDist. */
  public CMD_ReefAlign(SUB_Drivetrain drivetrain, SUB_PhotonVision photonVision, boolean isLeftAlign) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.photonVision = photonVision;
    this.isLeftAlign = isLeftAlign;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    var alliance = DriverStation.getAlliance();

    int[] targetTagSet;

    if (alliance.isPresent()) {
      if (alliance.get() == DriverStation.Alliance.Red) {
        targetTagSet = new int[] {6, 7, 8, 9, 10, 11};

      } else {
        targetTagSet = new int[] {17, 18, 19, 20, 21, 22};
      }
    } else {
      SmartDashboard.putBoolean("Alliance Error", true);
      end(true);
      return;
    }

    double minDistance = Double.MAX_VALUE;
    for (int tag : targetTagSet) {
      Pose2d pose = photonVision.at_field.getTagPose(tag).get().toPose2d();
      Translation2d translate = pose.minus(drivetrain.getPose()).getTranslation();
      double distance = Math.sqrt(Math.pow(translate.getX(), 2) + Math.pow(translate.getY(), 2));

      if (distance <= minDistance) {
        tagPose = pose;
      }
    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d currentPose = drivetrain.getPose();
    Translation2d tagRelativeTranslation = currentPose.minus(tagPose).getTranslation(); // Gets the
                                                                                        // "robot
                                                                                        // pose"
                                                                                        // relative
                                                                                        // to the
                                                                                        // tag
    double xSpeed = xController.calculate(tagRelativeTranslation.getX(), 0.5);
    double ySpeed = yController.calculate(tagRelativeTranslation.getY(), 0.5);
    double omegaSpeed = robotAngleController.calculate(currentPose.getRotation().getRadians(),
        tagPose.getRotation().getRadians());


    if (isLeftAlign){
    // + X is forward, + Y is to the left, + Theta is counterclockwise
    // Try 0, -ySpeed, 0 first.
      drivetrain.drive(xSpeed, -ySpeed, omegaSpeed, false, true);
    } else {
      drivetrain.drive(xSpeed, ySpeed, omegaSpeed, false, true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    SmartDashboard.putBoolean ("AlignCommandComplete!", xController.atSetpoint() && yController.atSetpoint() && robotAngleController.atSetpoint());
    return xController.atSetpoint() && yController.atSetpoint() && robotAngleController.atSetpoint();
  }
}
