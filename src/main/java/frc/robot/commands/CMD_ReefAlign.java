// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;
import java.util.Arrays;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.SUB_Drivetrain;
import frc.robot.subsystems.SUB_PhotonVision;

public class CMD_ReefAlign extends RunCommand {
  SUB_PhotonVision photonVision;
  SUB_Drivetrain drivetrain;

  Pose2d tagPose;
  Integer targetId;

  boolean isLeftAlign;
  List<Integer> targetTagSet;

  CommandXboxController driverController;

  private final PIDController xController = new PIDController(0.2, 0, .02);
  private final PIDController yController = new PIDController(0.2, 0, .02);
  private final PIDController robotAngleController = new PIDController(0.5, 0, 0.1);
  
  double xMagnitude = Units.inchesToMeters(15+48); //meters
  double yMagnitude = Units.inchesToMeters(24);
  
  /** Creates a new CMD_ReefAlign. */
  public CMD_ReefAlign(SUB_Drivetrain drivetrain, SUB_PhotonVision photonVision, boolean isLeftAlign) {
    // Use addRequirements() here to declare subsystem dependencies.
    super(() -> {}, drivetrain);
    this.drivetrain = drivetrain;
    this.photonVision = photonVision;
    this.isLeftAlign = isLeftAlign;
    robotAngleController.enableContinuousInput(-Math.PI, Math.PI);
    robotAngleController.setTolerance(Units.degreesToRadians(1));
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    xController.setTolerance(0);
    yController.setTolerance(0);
    robotAngleController.setTolerance(0);
    var alliance = DriverStation.getAlliance();

    if (alliance.isPresent()) {
      if (alliance.get() == DriverStation.Alliance.Red) {
        targetTagSet =  Arrays.asList(new Integer[]{7, 8, 9, 10, 11, 6});
      } else {
        targetTagSet =  Arrays.asList(new Integer[]{21, 20, 19, 18, 17, 22});
      }
    } else {
      SmartDashboard.putBoolean("Alliance Error", true);
      end(true);
      return;
    }

    double minDistance = Double.MAX_VALUE;
    for (int tag : new int[]{6}) {
      Pose2d pose = photonVision.at_field.getTagPose(tag).get().toPose2d();
      Translation2d translate = pose.minus(drivetrain.getPose()).getTranslation();
      double distance = Math.sqrt(Math.pow(translate.getX(), 2) + Math.pow(translate.getY(), 2));

      if (distance <= minDistance) {
        tagPose = pose;
        targetId = tag;
      }
    }
    robotAngleController.reset();
    xController.reset();
    yController.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d currentPose = drivetrain.getPose();

    double angle = Units.degreesToRadians(60*targetTagSet.indexOf(targetId));

    double offSet = 90;
    if (!isLeftAlign) {
      offSet *= -1;
    }
    offSet = Units.degreesToRadians(offSet);

    

    double x = xMagnitude*Math.cos(angle) + yMagnitude*Math.cos(angle+offSet);
    double y = xMagnitude*Math.sin(angle) + yMagnitude*Math.sin(angle+offSet);


  
 
    SmartDashboard.putNumber("X CURRENT", drivetrain.getPose().getX());
    SmartDashboard.putNumber("Y CURRENT", drivetrain.getPose().getY());

    drivetrain.publisher1.set(new Pose2d(tagPose.getX() + x, tagPose.getY() + y, tagPose.getRotation()));

    double xSpeed = xController.calculate(drivetrain.getPose().getX(), tagPose.getX() + x);
    double ySpeed = yController.calculate(drivetrain.getPose().getY(), tagPose.getY() + y);
    double omegaSpeed = robotAngleController.calculate(MathUtil.angleModulus(currentPose.getRotation().getRadians()), MathUtil.angleModulus(tagPose.getRotation().getRadians()));
    
    SmartDashboard.putNumber("THETA INPUT", MathUtil.angleModulus(currentPose.getRotation().getRadians()));
    SmartDashboard.putNumber("THETA TARGET",MathUtil.angleModulus(tagPose.getRotation().getRadians()));
    drivetrain.drive(xSpeed, ySpeed, -omegaSpeed, true, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    SmartDashboard.putBoolean ("AlignCommandComplete!", xController.atSetpoint() && yController.atSetpoint() && robotAngleController.atSetpoint());
    return xController.atSetpoint() && yController.atSetpoint();
  }
}
