// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.Elevator;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.Roller;
import frc.robot.subsystems.*;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static SUB_Drivetrain drivetrain = SUB_Drivetrain.getInstance();
  public static SUB_Elevator elevator = SUB_Elevator.getInstance();
  public static SUB_Roller roller = SUB_Roller.getInstance();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController Driver1 =
      new CommandXboxController(OperatorConstants.kDriver1ControllerPort);

  private final CommandXboxController Driver2 =
      new CommandXboxController(OperatorConstants.kDriver2ControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    drivetrain.setDefaultCommand(new RunCommand(() -> drivetrain.drive(
        -MathUtil.applyDeadband(Driver1.getRawAxis(1), OperatorConstants.kDriveDeadband),
        -MathUtil.applyDeadband(Driver1.getRawAxis(0), OperatorConstants.kDriveDeadband),
        -MathUtil.applyDeadband(Driver1.getRawAxis(4), OperatorConstants.kDriveDeadband), true,
        true), drivetrain));

    elevator.setDefaultCommand(new RunCommand(() -> elevator.RunElevatorManual(0.0), elevator));
    roller.setDefaultCommand(new RunCommand(() -> roller.setOutput(0), roller));

    Driver1.povDown()
        .whileTrue(new RunCommand(() -> drivetrain.drive(
            -MathUtil.applyDeadband(
                Math.copySign(Math.pow(Driver1.getRawAxis(1), 2), Driver1.getRawAxis(1)),
                OperatorConstants.kDriveDeadband),
            -MathUtil.applyDeadband(
                Math.copySign(Math.pow(Driver1.getRawAxis(0), 2), Driver1.getRawAxis(0)),
                OperatorConstants.kDriveDeadband),
            -MathUtil.applyDeadband(Driver1.getRawAxis(4), OperatorConstants.kDriveDeadband), false,
            true), drivetrain));

    // Configure the trigger bindings
    configureBindings();
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

    Driver2.a().onTrue(new InstantCommand(() -> elevator.ChangeSetpoint(Elevator.kL1Setpoint)));
    Driver2.b().onTrue(new InstantCommand(() -> elevator.ChangeSetpoint(Elevator.kL2Setpoint)));
    Driver2.x().onTrue(new InstantCommand(() -> elevator.ChangeSetpoint(Elevator.kL3Setpoint)));
    Driver2.y().onTrue(new InstantCommand(() -> elevator.ChangeSetpoint(Elevator.kL4Setpoint)));

    Driver2.leftBumper()
        .toggleOnTrue(new RunCommand(() -> roller.setOutput(Roller.kIntakeSpeed), roller)
            .until(roller.outputCurrentThreshold())
            .andThen(new ParallelCommandGroup(
              new RunCommand(() -> roller.setOutput(Roller.kIntakeFinishSpeed)),
              new InstantCommand(() -> Driver1.getHID().setRumble(RumbleType.kBothRumble,1)),
              new InstantCommand(() -> Driver2.getHID().setRumble(RumbleType.kBothRumble,1)),
              new InstantCommand(() -> roller.hasCoral(true))
              ).withTimeout(Roller.kIntakeFinishTime)));
    Driver2.rightBumper().whileTrue(new RunCommand(()->roller.setOutput(Roller.kEjectSpeed), roller).finallyDo(()->roller.hasCoral(false)));
    Driver1.leftTrigger().whileTrue(new RunCommand(() -> elevator.RunElevatorManual(0.2), elevator));
    Driver1.rightTrigger().whileTrue(new RunCommand(() -> elevator.RunElevatorManual(-0.2), elevator));
  }



  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
