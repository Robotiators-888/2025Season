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

/**
 * The CMD_L1Align class is a command that aligns the robot to a Level 1 scoring position.
 * It determines the closest AprilTag and uses PathPlanner to generate a path to a hardcoded
 * target pose corresponding to that tag. The target poses are different for each alliance
 * color and for left/right alignment preferences.
 */
public class CMD_L1Align extends Command {

  /** The PathPlanner command that will be generated and executed. */
  private Command pathfindingCommand;

  /** A boolean indicating whether to align to the left or right side of the target. */
  private boolean isLeftAlign = false;

  /** The PhotonVision subsystem instance for AprilTag detection. */
  private SUB_PhotonVision photonVision;

  /** The drivetrain subsystem instance for robot movement. */
  private SUB_Drivetrain drivetrain;

  /** A map of hardcoded coordinates for the left side of the red alliance L1 scoring positions. */
  private HashMap<Integer, Translation2d> redLeft = new HashMap<>();

  /** A map of hardcoded coordinates for the right side of the red alliance L1 scoring positions. */
  private HashMap<Integer, Translation2d> redRight = new HashMap<>();

  /** A map of hardcoded coordinates for the left side of the blue alliance L1 scoring positions. */
  private HashMap<Integer, Translation2d> blueLeft = new HashMap<>();

  /** A map of hardcoded coordinates for the right side of the blue alliance L1 scoring positions. */
  private HashMap<Integer, Translation2d> blueRight = new HashMap<>();

  /**
   * Creates a new CMD_L1Align command.
   * @param drivetrain The drivetrain subsystem to use.
   * @param photonVision The PhotonVision subsystem to use.
   * @param isLeftAlign True to align to the left side of the target, false to align to the right.
   */
  public CMD_L1Align(SUB_Drivetrain drivetrain, SUB_PhotonVision photonVision,
      boolean isLeftAlign) {
    this.photonVision = photonVision;
    this.drivetrain = drivetrain;
    this.isLeftAlign = isLeftAlign;

    // Populate the HashMaps with hardcoded coordinates for each L1 scoring position,
    // keyed by AprilTag ID.
    redRight.put(7, new Translation2d(14.341348, 4.43982));
    redLeft.put(7, new Translation2d(14.341348, 3.6018200000000005));
    redRight.put(8, new Translation2d(13.33700635581432, 5.340349553296214));
    redLeft.put(8, new Translation2d(14.062735644185679, 4.9213495532962135));
    redRight.put(9, new Translation2d(12.05506835581432, 4.9213495532962135));
    redLeft.put(9, new Translation2d(12.780797644185679, 5.340349553296214));
    redRight.put(10, new Translation2d(11.776455999999998, 3.6018200000000005));
    redLeft.put(10, new Translation2d(11.776455999999998, 4.43982));
    redRight.put(11, new Translation2d(12.780797644185679, 2.7012904467037853));
    redLeft.put(11, new Translation2d(12.05506835581432, 3.1202904467037853));
    redRight.put(6, new Translation2d(14.062735644185679, 3.1202904467037853));
    redLeft.put(6, new Translation2d(13.33700635581432, 2.7012904467037853));
    blueRight.put(21, new Translation2d(5.771896, 4.43982));
    blueLeft.put(21, new Translation2d(5.771896, 3.6018200000000005));
    blueRight.put(20, new Translation2d(4.76730035581432, 5.340349553296214));
    blueLeft.put(20, new Translation2d(5.49302964418568, 4.9213495532962135));
    blueRight.put(19, new Translation2d(3.4856163558143196, 4.9213495532962135));
    blueLeft.put(19, new Translation2d(4.211345644185679, 5.340349553296214));
    blueRight.put(18, new Translation2d(3.20675, 3.6018200000000005));
    blueLeft.put(18, new Translation2d(3.20675, 4.43982));
    blueRight.put(17, new Translation2d(4.211345644185679, 2.7012904467037853));
    blueLeft.put(17, new Translation2d(3.4856163558143187, 3.1202904467037853));
    blueRight.put(22, new Translation2d(5.49302964418568, 3.1202904467037853));
    blueLeft.put(22, new Translation2d(4.76730035581432, 2.7012904467037853));
    
    
    addRequirements(drivetrain);
  }

  /**
   * Called when the command is initially scheduled. This method determines the closest AprilTag,
   * selects the appropriate hardcoded target pose based on alliance and alignment preference,
   * and generates a PathPlanner command to drive to that pose.
   */
  @Override
  public void initialize() {
    Pose2d tagPose = new Pose2d();
    Integer targetId = 7;

    List<Integer> targetTagSet;
    Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
    HashMap<Integer, Translation2d> selectedMap;

    // Determine which set of tags and coordinates to use based on alliance and alignment preference.
    if (alliance.isPresent()) {
      targetTagSet =
          alliance.get() == DriverStation.Alliance.Red ? Arrays.asList(7, 8, 9, 10, 11, 6)
              : Arrays.asList(21, 20, 19, 18, 17, 22);

      if (isLeftAlign) {
        selectedMap = alliance.get() == DriverStation.Alliance.Red ? redLeft : blueLeft;
      } else {
        selectedMap = alliance.get() == DriverStation.Alliance.Red ? redRight : blueRight;
      }
    } else {
      // If alliance is not present, cannot proceed.
      return;
    }

    // Find the closest AprilTag to the robot.
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

    // Define PathPlanner constraints.
    PathConstraints constraints = new PathConstraints(
    3.0, 2.5,
    Units.degreesToRadians(540), Units.degreesToRadians(720));

    // Get the target translation from the selected map and create the final target pose.
    Translation2d translate = selectedMap.get(targetId);
    Pose2d pose = new Pose2d(translate.getX(), translate.getY(), tagPose.getRotation().plus(Rotation2d.fromRadians(Math.PI)));
    drivetrain.publisher1.set(pose);

    // Build the pathfinding command.
    pathfindingCommand = AutoBuilder.pathfindToPose(pose, constraints);

    pathfindingCommand.initialize();
  }

  /**
   * Called every time the scheduler runs while the command is scheduled.
   * Executes the generated PathPlanner command.
   */
  @Override
  public void execute() {
    pathfindingCommand.execute();
  }

  /**
   * Called once the command ends or is interrupted.
   * @param interrupted True if the command was interrupted, false otherwise.
   */
  @Override
  public void end(boolean interrupted) {
    pathfindingCommand.end(interrupted);
  }

  /**
   * Returns true when the command should end.
   * @return True when the pathfinding command is finished, false otherwise.
   */
  @Override
  public boolean isFinished() {
    return pathfindingCommand.isFinished();
  }
}