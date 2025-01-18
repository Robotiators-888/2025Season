// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Elevator;


public class SUB_Elevator extends SubsystemBase {
  private static SUB_Elevator INSTANCE = null;
  private static boolean HasHomed = Elevator.kStartingHoming;
  private static SparkMax primary = new SparkMax(0, MotorType.kBrushless);
  private static SparkMax secondary = new SparkMax(0, MotorType.kBrushless);
  private static SparkMaxConfig config = new SparkMaxConfig();
  private static RelativeEncoder primaryencoder = primary.getEncoder();
  private static PIDController elevatorPID =
      new PIDController(Elevator.kP, Elevator.kI, Elevator.kD);
  private static SparkLimitSwitch LowerLimitSwitch = primary.getForwardLimitSwitch();

  private SUB_Elevator() {
    config.follow(primary);
    config.inverted(true);
    secondary.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    config.inverted(false);
    config.disableFollowerMode();
    primary.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  public static void RunElevatorManual(double manual) {
    primary.set(manual);
  }

  public static void RunElevator() {
    double movementspeed = elevatorPID.calculate(primaryencoder.getPosition());
    primary.set(movementspeed);

    if (primaryencoder.getPosition() > Elevator.kResetHomingThreshold) {
      HasHomed = false;
      SmartDashboard.putBoolean("Homed", HasHomed);
      SmartDashboard.putBoolean("EMERGENCY HOMED!!!", false);
    }
  }

  public static void ChangeSetpoint(double setpoint) {
    elevatorPID.setSetpoint(setpoint);
    SmartDashboard.putNumber("Elevator Target Setpoint", setpoint);
  }


  public static void HomeElevator() {
    if (LowerLimitSwitch.isPressed()) {
      RunElevatorManual(0);
      primaryencoder.setPosition(Elevator.kHomingEncoderLocation);
      HasHomed = true;
      SmartDashboard.putBoolean("Homed", HasHomed);
      return;
    }
    if (!LowerLimitSwitch.isPressed() && primaryencoder.getPosition() < Elevator.kEncoderNearZero
        && primary.getOutputCurrent() > Elevator.kHomingEmergencyCurrent) {
      primaryencoder.setPosition(Elevator.kHomingEncoderLocation);
      SmartDashboard.putBoolean("EMERGENCY HOMED!!!", true);
      RunElevatorManual(0);
      return;
    } else {
      RunElevator();
    }
  }

  public static SUB_Elevator getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new SUB_Elevator();
    }

    return INSTANCE;
  }

  public void periodic() {}
}
