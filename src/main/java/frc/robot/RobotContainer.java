// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.*;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static SUB_Drivetrain drivetrain = SUB_Drivetrain.getInstance();
  public static SUB_Shooter shooter = SUB_Shooter.getInstance();
  public static SUB_Index index = SUB_Index.getInstance();
  public static SUB_Intake intake = SUB_Intake.getInstance();
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
  // Manual setpoint shooter Up and Down
  Driver2.leftBumper().onTrue(
    new InstantCommand(() -> SUB_Shooter.MANUAL_RPM -= 250));// Decrease manual RPM by 250
  Driver2.rightBumper().onTrue(
    new InstantCommand(() -> SUB_Shooter.MANUAL_RPM += 250)); // Increase manual RPM by 250
   
    Driver2.rightTrigger().whileTrue(
                new ParallelCommandGroup(
                        new InstantCommand(() -> index.starttimer()),
                        new RunCommand(() -> index.setMotorSpeed(-Constants.Intake.kIndexSpeed), index),
                        new RunCommand(() -> shooter.shootFlywheelOnRPM(-1000), shooter)).until(
                                () -> index.CurrentLimitSpike())
                        .andThen(
                                new RunCommand(() -> index.setMotorSpeed(-0.05)).withTimeout(0.025))
                        .andThen(
                                new ParallelCommandGroup(
                                        new InstantCommand(() -> intake.setHasNote(true)),
                                        new InstantCommand(() -> index.setMotorSpeed(0)),
                                        new InstantCommand(() -> shooter.setMotorSpeed(0))).andThen(
                                                new SequentialCommandGroup(
                                                        new WaitCommand(.5),
                                                        new ParallelCommandGroup(
                                                                new InstantCommand(
                                                                        () -> Driver1.getHID().setRumble(
                                                                                GenericHID.RumbleType.kBothRumble, 0)),
                                                                new InstantCommand(
                                                                        () -> Driver2.getHID().setRumble(
                                                                                GenericHID.RumbleType.kBothRumble, 0)))))))
                .onFalse(
                        new ParallelCommandGroup(
                                new RunCommand(() -> shooter.setMotorSpeed(0.0), shooter),
                                new RunCommand(() -> index.setMotorSpeed(0.0), index))); // Spin Shooter IN
      // Spin manual shooter
        Driver2.back().whileTrue(
                new ParallelCommandGroup(
                        new RunCommand(() -> shooter.shootFlywheelOnRPM(SUB_Shooter.SetpointRPM), shooter),
                        new SequentialCommandGroup(
                                new WaitUntilCommand(() -> shooter.getFlywheelRPM() >= SUB_Shooter.SetpointRPM - 150),
                                new RunCommand(() -> index.setMotorSpeed(0.5), index),
                                new InstantCommand(() -> intake.setHasNote(false)))))
                .onFalse( 
                        new ParallelCommandGroup(
                                new InstantCommand(() -> intake.setHasNote(false)),
                                new InstantCommand(() -> index.setMotorSpeed(0)),
                                new InstantCommand(() -> shooter.setMotorSpeed(0))));
     // Intake button
     Driver2.a().whileTrue(
      new ParallelCommandGroup( 
              new InstantCommand(() -> index.starttimer()),
              new RunCommand(() -> index.setMotorSpeed(Constants.Intake.kIndexSpeed), index),
              new RunCommand(() -> intake.setMotorSpeed(Constants.Intake.kIntakingSpeed))).until(
                      () -> index.CurrentLimitSpike())
              .andThen(
                      new InstantCommand(
                              () -> Driver1.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 1)),
                      new InstantCommand(
                              () -> Driver2.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 1)))
              .andThen(
                      new InstantCommand(() -> intake.setHasNote(true)),
                      new RunCommand(() -> index.setMotorSpeed(-0.1)).withTimeout(0.1).andThen(
                              new ParallelCommandGroup(
                                      new InstantCommand(() -> index.setMotorSpeed(0)),
                                      new InstantCommand(() -> shooter.setMotorSpeed(0))))))
      .onFalse(
              new ParallelCommandGroup(
                      new InstantCommand(() -> index.setMotorSpeed(0)),
                      new InstantCommand(() -> intake.setMotorSpeed(0)),
                      new InstantCommand(() -> shooter.shootFlywheelOnRPM(1500))).andThen(
                              new SequentialCommandGroup(
                                      new WaitCommand(.5),
                                      new ParallelCommandGroup(
                                              new InstantCommand(() -> Driver1.getHID()
                                                      .setRumble(GenericHID.RumbleType.kBothRumble, 0)),
                                              new InstantCommand(() -> Driver2.getHID()
                                                      .setRumble(GenericHID.RumbleType.kBothRumble, 0))),
                                      new WaitCommand(1.0))));

    // Outtake button
    Driver2.x().whileTrue(
      new ParallelCommandGroup(
              new RunCommand(() -> index.setMotorSpeed(-0.6), index),
              new RunCommand(() -> intake.setMotorSpeed(-0.6), intake)))
      .onFalse(
              new ParallelCommandGroup(
                      new InstantCommand(() -> index.setMotorSpeed(0)),
                      new InstantCommand(() -> intake.setMotorSpeed(0))));

   // Drive Intake OUT
   Driver2.leftTrigger()
   .whileTrue(new RunCommand(() -> intake.setMotorSpeed(-Constants.Intake.kOutakeSpeed), intake))
   .onFalse(new InstantCommand(() -> intake.setMotorSpeed(0.0)));


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
