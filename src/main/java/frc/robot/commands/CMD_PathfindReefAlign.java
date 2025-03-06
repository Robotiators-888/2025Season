// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SUB_Drivetrain;
import frc.robot.subsystems.SUB_PhotonVision;

/*
 * You should consider using the more terse Command factories API instead
 * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#
 * defining-commands
 */
public class CMD_PathfindReefAlign extends Command {

  Command pathfindingCommand;
  boolean isLeftAlign = false;
  SUB_PhotonVision photonVision;
  SUB_Drivetrain drivetrain;

  HashMap<Integer, Translation2d> redLeft = new HashMap<>();
  HashMap<Integer, Translation2d> redRight = new HashMap<>();
  HashMap<Integer, Translation2d> blueLeft = new HashMap<>();
  HashMap<Integer, Translation2d> blueRight = new HashMap<>();

  /** Creates a new CMD_PathfindReefAlign. */
  public CMD_PathfindReefAlign(SUB_Drivetrain drivetrain, SUB_PhotonVision photonVision,
      boolean isLeftAlign) {
    this.photonVision = photonVision;
    this.drivetrain = drivetrain;
    this.isLeftAlign = isLeftAlign;

    redRight.put(7, new Translation2d(14.392147999999999, 4.2116375));
    redLeft.put(7, new Translation2d(14.392147999999999, 3.8401625));
    redRight.put(8, new Translation2d(13.56441760656459, 5.272792393808464));
    redLeft.put(8, new Translation2d(13.886124393435413, 5.087054893808464));
    redRight.put(9, new Translation2d(12.231679606564587, 5.087054893808464));
    redLeft.put(9, new Translation2d(12.55338639343541, 5.272792393808464));
    redRight.put(10, new Translation2d(11.725655999999999, 3.8401625));
    redLeft.put(10, new Translation2d(11.725655999999999, 4.2116375));
    redRight.put(11, new Translation2d(12.55338639343541, 2.779007606191536));
    redLeft.put(11, new Translation2d(12.231679606564587, 2.964745106191536));
    redRight.put(6, new Translation2d(13.886124393435413, 2.964745106191536));
    redLeft.put(6, new Translation2d(13.56441760656459, 2.779007606191536));
    blueRight.put(21, new Translation2d(5.822696, 4.2116375));
    blueLeft.put(21, new Translation2d(5.822696, 3.8401625));
    blueRight.put(20, new Translation2d(4.994711606564587, 5.272792393808464));
    blueLeft.put(20, new Translation2d(5.316418393435412, 5.087054893808464));
    blueRight.put(19, new Translation2d(3.662227606564587, 5.087054893808464));
    blueLeft.put(19, new Translation2d(3.9839343934354114, 5.272792393808464));
    blueRight.put(18, new Translation2d(3.15595, 3.8401625));
    blueLeft.put(18, new Translation2d(3.15595, 4.2116375));
    blueRight.put(17, new Translation2d(3.983934393435411, 2.779007606191536));
    blueLeft.put(17, new Translation2d(3.6622276065645867, 2.964745106191536));
    blueRight.put(22, new Translation2d(5.316418393435412, 2.964745106191536));
    blueLeft.put(22, new Translation2d(4.994711606564588, 2.779007606191536));
    

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
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

    PathConstraints constraints = new PathConstraints(
    3.0, 2.1,
    Units.degreesToRadians(540), Units.degreesToRadians(720));


    Translation2d translate = selectedMap.get(targetId);
    Pose2d pose = new Pose2d(translate.getX(), translate.getY(), tagPose.getRotation().plus(Rotation2d.fromRadians(Math.PI)));
    drivetrain.publisher1.set(pose);
    pathfindingCommand = AutoBuilder.pathfindToPose(pose, constraints);

    pathfindingCommand.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pathfindingCommand.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pathfindingCommand.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pathfindingCommand.isFinished();
  }
}
