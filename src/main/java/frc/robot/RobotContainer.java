// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Field;
import frc.robot.Constants.Operator;
import frc.robot.commands.CMD_ReefAlign;
import frc.robot.subsystems.SUB_Drivetrain;
import frc.robot.subsystems.SUB_PhotonVision;
import frc.robot.utils.AllianceFlipUtil;
import frc.robot.utils.AutoGenerator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.PowerDistribution;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private static final SUB_Drivetrain drivetrain = SUB_Drivetrain.getInstance();
  private static final SUB_PhotonVision photonVision = SUB_PhotonVision.getInstance();
  private static final AutoGenerator autoGenerator = AutoGenerator.getInstance();
  private final SendableChooser<Command> autoChooser;
  public static PowerDistribution powerDistribution = new PowerDistribution();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController Driver1 =
      new CommandXboxController(Operator.kDriver1ControllerPort);

  private final CommandXboxController Driver2 =
      new CommandXboxController(Operator.kDriver2ControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    drivetrain.setDefaultCommand(new RunCommand( //Unstable
        () -> drivetrain.drive(
            MathUtil.applyDeadband(Driver1.getRawAxis(1), Operator.kDriveDeadband),
            MathUtil.applyDeadband(Driver1.getRawAxis(0), Operator.kDriveDeadband),
            -MathUtil.applyDeadband(Driver1.getRawAxis(4), Operator.kDriveDeadband), true, true),
        drivetrain));

        
    Driver1.povDown()
        .whileTrue(new RunCommand(
            () -> drivetrain.drive(
                -MathUtil.applyDeadband(
                    Math.copySign(Math.pow(Driver1.getRawAxis(1), 2), Driver1.getRawAxis(1)),
                    Operator.kDriveDeadband),
                -MathUtil.applyDeadband(
                    Math.copySign(Math.pow(Driver1.getRawAxis(0), 2), Driver1.getRawAxis(0)),
                    Operator.kDriveDeadband),
                -MathUtil.applyDeadband(Driver1.getRawAxis(4), Operator.kDriveDeadband), false,
                true),
            drivetrain));

    File pathFolder = new File(Filesystem.getDeployDirectory()+"/pathplanner/paths/");
    File[] listOfFiles = pathFolder.listFiles();
    List<String> pathNames = new ArrayList<>();

    if (listOfFiles != null) {
      for (File file : listOfFiles) {
        if (file.isFile() && file.getName().endsWith(".path")) {
          pathNames.add(file.getName());
        }
      }
    }

    for (String pathName : pathNames) {
      String modifiedPathName = pathName.substring(0, pathName.length() - 5);
      NamedCommands.registerCommand(modifiedPathName+" Pathfind", getPathCommand(modifiedPathName));
    }
    // Configure the trigger bindings
    configureBindings();

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in
   * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings() {
    Driver1.leftBumper().whileTrue(new CMD_ReefAlign(drivetrain, photonVision, true));
    Driver1.rightBumper().whileTrue(new CMD_ReefAlign(drivetrain, photonVision, false));
    Driver1.leftStick().onTrue(new InstantCommand(() -> drivetrain.zeroHeading()));
  }

  public void robotInit() {
    powerDistribution.setSwitchableChannel(true);
  }


  public Command getPathCommand(String pathName) {
    Pathfinding.setPathfinder(new LocalADStar());
    try {
      PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
      PathConstraints constraints = new PathConstraints(
            0.5, 0.5,
            Units.degreesToRadians(180), Units.degreesToRadians(180)); //unstable
    return AutoBuilder.pathfindThenFollowPath(
     path,
    constraints);
    } catch (Exception e) {
      DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
      return Commands.none();
    }
  }




  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
//     return autoChooser.getSelected();
//     Pathfinding.setPathfinder(new LocalADStar());

//     try{
    // // Load the path we want to pathfind to and follow
//     PathPlannerPath path = PathPlannerPath.fromPathFile("New Path");
//     drivetrain.publisher1.set(path.getStartingHolonomicPose().get());
//     // // Create the constraints to use while pathfinding. The constraints defined in the path will only be used for the path.
//     PathConstraints constraints = new PathConstraints(
//             0.5, 0.5,
//             Units.degreesToRadians(180), Units.degreesToRadians(180));

//     // Since AutoBuilder is configured, we can use it to build pathfinding commands
//     return AutoBuilder.pathfindThenFollowPath(
//      path,
//     constraints);
    //return AutoBuilder.followPath(path);

      PathPlannerAuto auto = new PathPlannerAuto("New Auto");
      return auto;

    // PathPlannerPath path = PathPlannerPath.fromPathFile("Angle Path");
    
    // RobotConfig robotConfig = RobotConfig.fromGUISettings();
    // PathPlannerTrajectory traj = path.getIdealTrajectory(robotConfig).get();

    // drivetrain.resetPose(
    //   AllianceFlipUtil.apply(path.getStartingHolonomicPose().get())
    // );
    // return AutoBuilder.followPath(path);
//     } catch (Exception e) {
//         DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
//         return Commands.none();
//     }
  }


  public void robotPeriodic() {
    //photonPoseUpdate();
    
  }

  public void autonomousPeriodic() {
    
  }

  

  public void teleopPeriodic() {
    try {
      PathPlannerPath paths = PathPlannerPath.fromPathFile("New Path");
      drivetrain.publisher1.set(AllianceFlipUtil.apply(paths.getStartingHolonomicPose().get()));
    } catch (Exception e) {
      DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
    }
    SmartDashboard.putNumber("Raw X Speed", -MathUtil.applyDeadband(Driver1.getRawAxis(1), Operator.kDriveDeadband));
    SmartDashboard.putNumber("Raw Y Speed", -MathUtil.applyDeadband(Driver1.getRawAxis(0), Operator.kDriveDeadband));
  }

  public static void photonPoseUpdate() {
    Optional<EstimatedRobotPose> photonPoseOptional = photonVision.getCam1Pose();

    if (photonPoseOptional.isPresent()) {
      Pose3d photonPose = photonPoseOptional.get().estimatedPose;
      
      if (photonPose.getX() >= 0 && photonPose.getX() <= Field.fieldLength && photonPose.getY() >= 0
          && photonPose.getY() <= Field.fieldWidth && photonVision.getCam1BestTarget() != null) {

        Pose2d closestTag = photonVision.at_field
            .getTagPose(photonVision.getCam1BestTarget().getFiducialId()).get().toPose2d();
        Translation2d translate = closestTag.minus(photonPose.toPose2d()).getTranslation();

        double distance = translate.getNorm();
        double xStddev = distance * 1.5;
        double yStddev = xStddev * 4;
        double rotStddev = Units.degreesToRadians(120.0);
        drivetrain.publisher3.set(photonPose.toPose2d());
        drivetrain.m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(xStddev, yStddev, rotStddev));  
        drivetrain.addVisionMeasurement(photonPose.toPose2d(), photonPoseOptional.get().timestampSeconds);
      }
    }

    photonPoseOptional = photonVision.getCam2Pose();

    if (photonPoseOptional.isPresent()) {
      Pose3d photonPose = photonPoseOptional.get().estimatedPose;
      
      if (photonPose.getX() >= 0 && photonPose.getX() <= Field.fieldLength && photonPose.getY() >= 0
          && photonPose.getY() <= Field.fieldWidth && photonVision.getCam2BestTarget() != null) {

        Pose2d closestTag = photonVision.at_field
            .getTagPose(photonVision.getCam2BestTarget().getFiducialId()).get().toPose2d();
        Translation2d translate = closestTag.minus(photonPose.toPose2d()).getTranslation();

        double distance = translate.getNorm();
        double xStddev = distance * 1.5;
        double yStddev = xStddev * 4;
        double rotStddev = Units.degreesToRadians(120.0);
        drivetrain.publisher4.set(photonPose.toPose2d());
        drivetrain.m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(xStddev, yStddev,rotStddev));  
        drivetrain.addVisionMeasurement(photonPose.toPose2d(), photonPoseOptional.get().timestampSeconds);
      }
    }
  }
}
