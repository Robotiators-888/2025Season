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

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CMD_PathfindReefAlign extends Command {

  Command pathfindingCommand;
  boolean isLeftAlign = false;
  SUB_PhotonVision photonVision;
  SUB_Drivetrain drivetrain;

  HashMap<Integer, Pose2d> redLeft = new HashMap<>();
  HashMap<Integer, Pose2d> redRight = new HashMap<>();
  HashMap<Integer, Pose2d> blueLeft = new HashMap<>();
  HashMap<Integer, Pose2d> blueRight = new HashMap<>();

  /** Creates a new CMD_PathfindReefAlign. */
  public CMD_PathfindReefAlign(SUB_Drivetrain drivetrain, SUB_PhotonVision photonVision, boolean isLeftAlign) {
    this.photonVision = photonVision;
    this.drivetrain = drivetrain;
    this.isLeftAlign = isLeftAlign;

    redLeft.put(null, null);
    redLeft.put(null, null);
    redLeft.put(null, null);
    redLeft.put(null, null);
    redLeft.put(null, null);
    redLeft.put(null, null);

    redRight.put(null, null);
    redRight.put(null, null);
    redRight.put(null, null);
    redRight.put(null, null);
    redRight.put(null, null);
    redRight.put(null, null);

    blueLeft.put(null, null);
    blueLeft.put(null, null);
    blueLeft.put(null, null);
    blueLeft.put(null, null);
    blueLeft.put(null, null);
    blueLeft.put(null, null);

    blueRight.put(null, null);
    blueRight.put(null, null);
    blueRight.put(null, null);
    blueRight.put(null, null);
    blueRight.put(null, null);
    blueRight.put(null, null);

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
    HashMap<Integer, Pose2d> selectedMap;
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
    3.0, 4.0,
    Units.degreesToRadians(540), Units.degreesToRadians(720));


    pathfindingCommand = AutoBuilder.pathfindToPose(selectedMap.get(targetId), constraints);

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
