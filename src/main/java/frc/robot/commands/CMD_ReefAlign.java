// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Optional;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;

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

public class CMD_ReefAlign extends RunCommand {
  private final SUB_PhotonVision photonVision;
  private final SUB_Drivetrain drivetrain;
  private Integer targetId;

  private final boolean isLeftAlign;
  private List<Integer> targetTagSet;
  private Pose2d targetPose = new Pose2d();

  HashMap<Integer, Translation2d> redLeft = new HashMap<>();
  HashMap<Integer, Translation2d> redRight = new HashMap<>();
  HashMap<Integer, Translation2d> blueLeft = new HashMap<>();
  HashMap<Integer, Translation2d> blueRight = new HashMap<>();

  private final PIDController xController = new PIDController(0.3, 0, 0.03);
  private final PIDController yController = new PIDController(0.3, 0, 0.03);
  private final PIDController robotAngleController = new PIDController(0.7, 0, 0.05);

  private final double xMagnitude = Constants.Drivetrain.kXShiftMagnitude;
  private final double yMagnitude = Constants.Drivetrain.kYShiftMagnitude;

  public CMD_ReefAlign(SUB_Drivetrain drivetrain, SUB_PhotonVision photonVision,
      boolean isLeftAlign) {
    super(() -> {
    }, drivetrain);

    this.drivetrain = drivetrain;
    this.photonVision = photonVision;
    this.isLeftAlign = isLeftAlign;
    robotAngleController.enableContinuousInput(-Math.PI, Math.PI);
    robotAngleController.setTolerance(Units.degreesToRadians(1));


    redLeft.put(6, new Translation2d(13.564418, 2.7790076));
    redLeft.put(7, new Translation2d(14.392148, 3.8401625));
    redLeft.put(8, new Translation2d(13.886124, 5.0870549));
    redLeft.put(9, new Translation2d(12.553386, 5.2727924));
    redLeft.put(10, new Translation2d(11.725656, 4.2116375));
    redLeft.put(11, new Translation2d(12.23168, 2.9647451));

    redRight.put(6, new Translation2d(13.88612437, 2.9647451));
    redRight.put(7, new Translation2d(14.392148, 4.2116375));
    redRight.put(8, new Translation2d(13.56441763, 5.2727924));
    redRight.put(9, new Translation2d(12.23167963, 5.0870549));
    redRight.put(10, new Translation2d(11.725656, 3.8401625));
    redRight.put(11, new Translation2d(12.55338637, 2.7790076));

    blueLeft.put(17, new Translation2d(3.6622276, 2.9647451));
    blueLeft.put(18, new Translation2d(3.5617949, 3.4996182));
    blueLeft.put(19, new Translation2d(3.9839344, 5.2727924));
    blueLeft.put(20, new Translation2d(5.3164184, 5.0870549));
    blueLeft.put(21, new Translation2d(5.822696, 3.8401625));
    blueLeft.put(22, new Translation2d(4.9947116, 2.7790076));

    blueRight.put(17, new Translation2d(3.983934374, 2.7790076));
    blueRight.put(18, new Translation2d(3.927626384, 3.5641242));
    blueRight.put(19, new Translation2d(3.662227626, 5.0870549));
    blueRight.put(20, new Translation2d(4.994711626, 5.2727924));
    blueRight.put(21, new Translation2d(5.822696, 4.2116375));
    blueRight.put(22, new Translation2d(5.316418374, 2.9647451));
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    xController.setTolerance(0);
    yController.setTolerance(0);
    robotAngleController.setTolerance(0);

    Pose2d tagPose = new Pose2d();
    Integer targetId = 7;


    List<Integer> targetTagSet;
    Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
    HashMap<Integer, Translation2d> selectedMap;
    if (alliance.isPresent()) {
      targetTagSet =
          alliance.get() == DriverStation.Alliance.Red ? Arrays.asList(7, 8, 9, 10, 11, 6)
              : Arrays.asList(21, 20, 19, 18, 17, 22);

      if (isLeftAlign) {
        if (alliance.get() == DriverStation.Alliance.Red) {
          selectedMap = redLeft;
        } else {
          selectedMap = blueLeft;
        }
      } else {
        if (alliance.get() == DriverStation.Alliance.Red) {
          selectedMap = redRight;
        } else {
          selectedMap = blueRight;
        }
      }
    } else {
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


    Translation2d translate = selectedMap.get(targetId);
    targetPose = new Pose2d(translate.getX(), translate.getY(),
        tagPose.getRotation().plus(Rotation2d.fromRadians(Math.PI)));
    robotAngleController.reset();
    xController.reset();
    yController.reset();
  }

  @Override
  public void execute() {
    Pose2d currentPose = drivetrain.getPose();

    drivetrain.publisher1.set(targetPose);


    double xSpeed = xController.calculate(currentPose.getX(), targetPose.getX());
    double ySpeed = yController.calculate(currentPose.getY(), targetPose.getY());
    double omegaSpeed = robotAngleController.calculate(
        MathUtil.angleModulus(currentPose.getRotation().getRadians()),
        MathUtil.angleModulus(targetPose.getRotation().getRadians()));

    drivetrain.drive(xSpeed, ySpeed, omegaSpeed, true, true);
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
