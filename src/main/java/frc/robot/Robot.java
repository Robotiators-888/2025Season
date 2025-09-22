// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. This class extends LoggedRobot from
 * AdvantageKit to enable advanced logging capabilities. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  /** The command selected for the autonomous period. */
  private Command m_autonomousCommand;

  /** The robot's main container, which holds all subsystems, commands, and operator interface configuration. */
  private final RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up. It should be used for any
   * initialization code. It starts the data log and AdvantageKit logger, and then
   * instantiates the RobotContainer.
   */
  public Robot() {
    DataLogManager.start();
    Logger.start();
    
    // Instantiate our RobotContainer. This will perform all button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every 20 ms, regardless of the mode. Use this for items like diagnostics
   * that you want to run during disabled, autonomous, teleoperated, and test modes.
   *
   * <p>This runs after the mode-specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods. This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    m_robotContainer.robotPeriodic();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    // Intentionally left blank.
  }

  /** This function is called periodically during disabled mode. */
  @Override
  public void disabledPeriodic() {
    m_robotContainer.disabledPeriodic();
  }

  /**
   * This function is called once when the autonomous period begins.
   * It gets the selected autonomous command from the RobotContainer and schedules it.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // Schedule the autonomous command if one was selected.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    m_robotContainer.autonomousInit();
  }

  /** This function is called periodically during the autonomous period. */
  @Override
  public void autonomousPeriodic() {
    m_robotContainer.autonomousPeriodic();
  }

  /**
   * This function is called once when the teleoperated period begins.
   * It cancels the autonomous command to ensure it doesn't interfere with teleop control.
   */
  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_robotContainer.teleopInit();
  }
  

  /** This function is called periodically during the teleoperated period (operator control). */
  @Override
  public void teleopPeriodic() {
    m_robotContainer.teleopPeriodic();
  }

  /**
   * This function is called once when test mode begins.
   * It cancels all running commands.
   */
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    // Intentionally left blank.
  }

  /** This function is called once when the simulation is first started up. */
  @Override
  public void simulationInit() {
    // Intentionally left blank.
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
    // Intentionally left blank.
  }
}
