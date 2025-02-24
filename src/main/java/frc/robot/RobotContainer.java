// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.Climber;
import frc.robot.Constants.Elevator;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.Roller;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.PowerDistribution;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static SUB_Drivetrain drivetrain = SUB_Drivetrain.getInstance();
  public static SUB_Elevator elevator = SUB_Elevator.getInstance();
  public static SUB_Roller roller = SUB_Roller.getInstance();
  public static SUB_Pivot pivot = SUB_Pivot.getInstance(roller.getAbsoluteEncoder());
  public static SUB_Climber climber = SUB_Climber.getInstance();
  public static PowerDistribution powerDistribution = new PowerDistribution();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController Driver1 = new CommandXboxController(OperatorConstants.kDriver1ControllerPort);

  private final CommandXboxController Driver2 = new CommandXboxController(OperatorConstants.kDriver2ControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    drivetrain.setDefaultCommand(new RunCommand(() -> drivetrain.drive(
        -MathUtil.applyDeadband(Driver1.getRawAxis(1),
            OperatorConstants.kDriveDeadband),
        -MathUtil.applyDeadband(Driver1.getRawAxis(0),
            OperatorConstants.kDriveDeadband),
        -MathUtil.applyDeadband(Driver1.getRawAxis(4),
            OperatorConstants.kDriveDeadband),
        true, true), drivetrain));

    elevator.setDefaultCommand(new ConditionalCommand(new RunCommand(() -> elevator.runElevator(), elevator),
        new WaitCommand(0.0), () -> pivot.atSetpoint(PivotConstants.kElevatingSetpoint)));

    // pivot.setDefaultCommand(
    //     new RunCommand(() -> pivot.runPivot(() -> roller.hasCoral(), () -> false), pivot));
    pivot.setDefaultCommand(
      new RunCommand(() -> pivot.runPivot(() -> false), pivot));


    Driver1.povDown().whileTrue(new RunCommand(
        () -> drivetrain.drive(-MathUtil.applyDeadband(
            Math.copySign(Math.pow(Driver1.getRawAxis(1), 2),
                Driver1.getRawAxis(1)),
            OperatorConstants.kDriveDeadband),
            -MathUtil.applyDeadband(Math.copySign(
                Math.pow(Driver1.getRawAxis(0), 2),
                Driver1.getRawAxis(0)),
                OperatorConstants.kDriveDeadband),
            -MathUtil.applyDeadband(Driver1.getRawAxis(4),
                OperatorConstants.kDriveDeadband),
            false, true),
        drivetrain));

    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in
   * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
   * for
   * {@link CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers
   * or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    Driver1.leftStick().onTrue(new InstantCommand(() -> drivetrain.zeroHeading()));
    Driver1.leftTrigger().whileTrue(new RunCommand(
        () -> climber.setSpeed(Climber.kClimberPercentOutput)))
        .onFalse(new InstantCommand(() -> climber.setSpeed(0.0)));
    Driver1.leftBumper().whileTrue(new RunCommand(
        () -> climber.setSpeed(-Climber.kClimberPercentOutput)))
        .onFalse(new InstantCommand(() -> climber.setSpeed(0.0)));
    ;
    Driver1.leftStick().onTrue(new InstantCommand(() -> drivetrain.zeroHeading()));

    Driver1.y().onTrue(new InstantCommand(
        () -> pivot.changeSetpoint(PivotConstants.kElevatingSetpoint)));
    Driver1.b().onTrue(new InstantCommand(
        () -> pivot.changeSetpoint(PivotConstants.kIntakeSetpoint)));
    Driver1.x().onTrue(new InstantCommand(
        () -> pivot.changeSetpoint(PivotConstants.kAlgaeSetpoint)));
    Driver1.a().onTrue(new InstantCommand(
        () -> pivot.changeSetpoint(PivotConstants.kCoralSetpoint)));

    // Driver 2
    Driver2.rightBumper()
    .onTrue(new SequentialCommandGroup(
        new InstantCommand(() -> pivot.changeSetpoint(PivotConstants.kElevatingSetpoint)),
        new InstantCommand(() -> elevator.ChangeSetpoint(0.0))));

    Driver2.a()
        .onTrue(new SequentialCommandGroup(
            new InstantCommand(() -> pivot.changeSetpoint(PivotConstants.kElevatingSetpoint)),
            new InstantCommand(() -> elevator.ChangeSetpoint(Elevator.kL1Setpoint)),
            Commands.waitUntil(() -> elevator.atSetpoint(Elevator.kL1Setpoint)).andThen(()->pivot.changeSetpoint(PivotConstants.kL1Setpoint)))
            );

    Driver2.b()
        .onTrue(new SequentialCommandGroup(
            new InstantCommand(() -> pivot.changeSetpoint(PivotConstants.kElevatingSetpoint)),
            new InstantCommand(() -> elevator.ChangeSetpoint(Elevator.kL2Setpoint)),
            Commands.waitUntil(() -> elevator.atSetpoint(Elevator.kL2Setpoint)).andThen(()->pivot.changeSetpoint(PivotConstants.kL2Setpoint)))
            );

    Driver2.x()
        .onTrue(new SequentialCommandGroup(
            new InstantCommand(() -> pivot.changeSetpoint(PivotConstants.kElevatingSetpoint)),
            new InstantCommand(() -> elevator.ChangeSetpoint(Elevator.kL3Setpoint)),
            Commands.waitUntil(() -> elevator.atSetpoint(Elevator.kL3Setpoint)).andThen(()->pivot.changeSetpoint(PivotConstants.kL3Setpoint)))
            );
      
    Driver2.y()
        .onTrue(new SequentialCommandGroup(
            new InstantCommand(() -> pivot.changeSetpoint(PivotConstants.kElevatingSetpoint)),
            new InstantCommand(() -> elevator.ChangeSetpoint(Elevator.kL4Setpoint)),
            Commands.waitUntil(() -> elevator.atSetpoint(Elevator.kL4Setpoint)).andThen(()->pivot.changeSetpoint(PivotConstants.kL4Setpoint)))
            );


    // Driver2.leftBumper()
    //     .whileTrue(new InstantCommand(() -> roller.timerInteract(true))
    //         .andThen(new RunCommand(
    //             () -> roller.setRollerOutput(Roller.kIntakeSpeed),
    //             roller).until(() -> roller.atCurrentThresholdandTimerElapsed()))
    //         .andThen(new ParallelCommandGroup(
    //             new RunCommand(() -> roller.setRollerOutput(
    //                 Roller.kIntakeFinishSpeed), roller),
    //             new InstantCommand(
    //                 () -> roller.timerInteract(false)),
    //             new InstantCommand(() -> Driver1.getHID().setRumble(
    //                 RumbleType.kBothRumble, 1)),
    //             new InstantCommand(() -> Driver2.getHID().setRumble(
    //                 RumbleType.kBothRumble, 1)),
    //             new InstantCommand(() -> roller.hasCoral(true)))
    //             .withTimeout(Roller.kIntakeFinishTime)
    //             .andThen(new ParallelCommandGroup(
    //                 new InstantCommand(
    //                     () -> Driver1.getHID()
    //                         .setRumble(RumbleType.kBothRumble,
    //                             0)),
    //                 new InstantCommand(
    //                     () -> Driver2.getHID()
    //                         .setRumble(RumbleType.kBothRumble,
    //                             0)))))
    //         .andThen(new InstantCommand(() -> roller.setRollerOutput(0.),
    //             roller)))
    //     .onFalse(new InstantCommand(() -> roller.setRollerOutput(0.),
    //         roller));

    Driver2.leftBumper().whileTrue(new RunCommand(()->roller.setRollerOutput(Roller.kIntakeSpeed))).onFalse(new InstantCommand(()->roller.setRollerOutput(0)));
    Driver2.leftTrigger().whileTrue(new RunCommand(
        () -> roller.setRollerOutput(Roller.kEjectSpeed), roller)
        .until(roller.isFreeSpinning())
        .andThen(new InstantCommand(
            () -> roller.hasCoral(false)))
        .andThen(new InstantCommand(
            () -> roller.setRollerOutput(0.),
            roller)))
        .onFalse(new InstantCommand(() -> roller.setRollerOutput(0.),
            roller));

  }

  public void robotInit() {
    powerDistribution.setSwitchableChannel(true);
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
