// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * The Main class is the entry point for the robot application.
 * It is responsible for starting the robot's execution by calling the RobotBase.startRobot method.
 *
 * Do NOT add any static variables to this class, or any initialization at all.
 * Unless you know what you are doing, do not modify this file except to change the
 * main robot class passed to the startRobot call.
 */
public final class Main {
  /**
   * Private constructor to prevent instantiation of this utility class.
   */
  private Main() {}

  /**
   * The main initialization function. This is the first method called when the robot code is started.
   * It simply calls the WPILib `startRobot` method, which handles the lifecycle of the robot program.
   *
   * If you change your main robot class (e.g., from `Robot` to `MyNewRobot`), you must
   * change the parameter type here (e.g., `RobotBase.startRobot(MyNewRobot::new)`).
   * @param args Command line arguments (not used in FRC).
   */
  public static void main(String... args) {
    RobotBase.startRobot(Robot::new);
  }
}
