// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static SUB_Drivetrain drivetrain = SUB_Drivetrain.getInstance();
  public static SUB_PhotonVision photonVision = SUB_PhotonVision.getInstance();
  public static AutoGenerator autoGenerator = AutoGenerator.getInstance();
  private final SendableChooser<Command> autoChooser;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController Driver1 =
      new CommandXboxController(Operator.kDriver1ControllerPort);

  private final CommandXboxController Driver2 =
      new CommandXboxController(Operator.kDriver2ControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    drivetrain.setDefaultCommand(new RunCommand(
        () -> drivetrain.drive(
            -MathUtil.applyDeadband(Driver1.getRawAxis(1), Operator.kDriveDeadband),
            -MathUtil.applyDeadband(Driver1.getRawAxis(0), Operator.kDriveDeadband),
            MathUtil.applyDeadband(Driver1.getRawAxis(4), Operator.kDriveDeadband), true, true),
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
  }



  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    try{
      PathPlannerAuto auto = new PathPlannerAuto("Straight Auto");
      return auto;
    } catch (Exception e) {
        DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
        return Commands.none();
    }
  }


  public void robotPeriodic() {
    //photonPoseUpdate();
  }

  public void autonomousPeriodic() {

  }

  

  public void teleopPeriodic() {
    SmartDashboard.putNumber("Raw X Speed", -MathUtil.applyDeadband(Driver1.getRawAxis(1), Operator.kDriveDeadband));
    SmartDashboard.putNumber("Raw Y Speed", -MathUtil.applyDeadband(Driver1.getRawAxis(0), Operator.kDriveDeadband));
  }

  public static void photonPoseUpdate() {
    Optional<EstimatedRobotPose> photonPoseOptional = photonVision.getEstimatedGlobalPose();

    if (photonPoseOptional.isPresent()) {
      Pose3d photonPose = photonPoseOptional.get().estimatedPose;
      if (photonPose.getX() >= 0 && photonPose.getX() <= Field.fieldLength && photonPose.getY() >= 0
          && photonPose.getY() <= Field.fieldWidth) {
      }

      if (photonVision.getBestTarget() == null) {
        return;
      }

      Pose2d closestTag = photonVision.at_field
          .getTagPose(photonVision.getBestTarget().getFiducialId()).get().toPose2d();
      Translation2d translate = closestTag.minus(photonPose.toPose2d()).getTranslation();

      double distance = Math.sqrt(Math.pow(translate.getX(), 2) + Math.pow(translate.getY(), 2));
      double xStddev = distance / 1.0;
      double yStddev = xStddev * 4;
      double rotStddev = Units.degreesToRadians(70.0);


      drivetrain.m_poseEstimator
          .setVisionMeasurementStdDevs(VecBuilder.fill(xStddev, yStddev, rotStddev));

      SmartDashboard.putNumberArray("PHOTON/Pose", new Double[] {photonPose.toPose2d().getX(),
          photonPose.toPose2d().getY(), photonPose.toPose2d().getRotation().getDegrees()});
      SmartDashboard.putNumberArray("PHOTON/Pose3d",
          new Double[] {photonPose.getX(), photonPose.getY(), photonPose.getZ(),
              photonPose.getRotation().getQuaternion().getW(),
              photonPose.getRotation().getQuaternion().getX(),
              photonPose.getRotation().getQuaternion().getY(),
              photonPose.getRotation().getQuaternion().getZ()});
      drivetrain.addVisionMeasurement(photonPose.toPose2d(),
          photonPoseOptional.get().timestampSeconds);
    }
  }
}
