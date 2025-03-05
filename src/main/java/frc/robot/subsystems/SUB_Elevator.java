// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Elevator;


public class SUB_Elevator extends SubsystemBase {
  private static SUB_Elevator INSTANCE = null;
  private double activesetpoint = 0;
  private SparkMax primary = new SparkMax(35, MotorType.kBrushless);
  private SparkMax secondary = new SparkMax(36, MotorType.kBrushless);
  private SparkMaxConfig config = new SparkMaxConfig();
  private RelativeEncoder primaryencoder = primary.getEncoder();
  private SparkLimitSwitch lowerLimitSwitch = primary.getReverseLimitSwitch();
  public static SUB_Roller roller = SUB_Roller.getInstance();


  private SUB_Elevator() {
    config.follow(primary);
    config.inverted(false);
    config.idleMode(IdleMode.kBrake);
    secondary.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    config.inverted(true);
    config.disableFollowerMode();
    config.encoder.positionConversionFactor((0.2 * Units.inchesToMeters(1.92 * Math.PI)));
    config.encoder.velocityConversionFactor((0.2 * Units.inchesToMeters(1.92 * Math.PI)) / 60);
    primary.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void runElevatorManualVoltage(double volts) {
    primary.setVoltage(volts);
  }

  public void zeroEncoder() {
    primaryencoder.setPosition(0);
  }

  public void runElevator(Supplier<Boolean> pivotSafe) {
    if (getCurrentPosition() >= Elevator.kResetHomingThreshold) {
      SmartDashboard.putBoolean("EMERGENCY HOMED!!!", false);
      SmartDashboard.putBoolean("Homed", false);
    }

    if (!pivotSafe.get() && !this.atSetpoint()){
      SmartDashboard.putBoolean("Elevator is Safe", false);
      if (roller.getHasCoral()) {
        runElevatorManualVoltage(Elevator.kCoralHoldingVoltage);
        return;
      }
      if (roller.getHasAlgae()) {
        runElevatorManualVoltage(Elevator.kAlgaeHoldingVoltage);
        return;
      }
      if (primaryencoder.getPosition() > .6) {
        runElevatorManualVoltage(Elevator.kEmptyHoldingVoltageTop);
        return;
      }
      runElevatorManualVoltage(Elevator.kEmptyHoldingVoltage);
      return;
    }

    SmartDashboard.putBoolean("Elevator is Safe", false);
    if (activesetpoint <= 0 && getCurrentPosition() <= Elevator.kMediumDownErrorThreshold) {
      HomeElevator();
      return;
    }

    if (Math.abs(activesetpoint - primaryencoder.getPosition()) < .02) {
      if (roller.getHasCoral()) {
        runElevatorManualVoltage(Elevator.kCoralHoldingVoltage);
        return;
      }
      if (roller.getHasAlgae()) {
        runElevatorManualVoltage(Elevator.kAlgaeHoldingVoltage);
        return;
      }
      if (primaryencoder.getPosition() > .6) {
        runElevatorManualVoltage(Elevator.kEmptyHoldingVoltageTop);
        return;
      }
      runElevatorManualVoltage(Elevator.kEmptyHoldingVoltage);
      return;
    }

    if (activesetpoint > getCurrentPosition()) {
      if (activesetpoint - getCurrentPosition() > Elevator.kMaxUpErrorThreshold) {
        runElevatorManualVoltage(Elevator.kMaxUpVoltage);
        return;
      }
      if (activesetpoint - getCurrentPosition() > Elevator.kHighUpErrorThreshold) {
        runElevatorManualVoltage(Elevator.kHighUpVoltage);
        return;
      }
      if (activesetpoint - getCurrentPosition() > Elevator.kMediumUpErrorThreshold) {
        runElevatorManualVoltage(Elevator.kMediumUpVoltage);
        return;
      }
      runElevatorManualVoltage(Elevator.kSlowUpVoltage);
      return;
    }
    if (activesetpoint < getCurrentPosition()) {
      if (Math.abs(getCurrentPosition() - activesetpoint) > Elevator.kMaxDownErrorThreshold) {
        runElevatorManualVoltage(Elevator.kMaxDownVoltage);
        return;
      }
      if (Math.abs(getCurrentPosition() - activesetpoint) > Elevator.kHighDownErrorThreshold) {
        runElevatorManualVoltage(Elevator.kHighDownVoltage);
        return;
      }
      if (Math.abs(getCurrentPosition() - activesetpoint) > Elevator.kMediumDownErrorThreshold) {
        runElevatorManualVoltage(Elevator.kMediumDownVoltage);
        return;
      }
      runElevatorManualVoltage(Elevator.kSlowDownVoltage);
      return;
    }
    runElevatorManualVoltage(0);
  }

  public boolean atSetpoint(double setpoint) {
    return Math.abs(primaryencoder.getPosition() - setpoint) < Elevator.kTolerance;
  }

  public boolean atSetpoint() {
    return atSetpoint(activesetpoint);
  }
  public double getCurrentPosition() {
    return primaryencoder.getPosition();
  }

  public void ChangeSetpoint(double setpoint) {
    activesetpoint = setpoint;
    SmartDashboard.putNumber("Elevator Target Setpoint", setpoint);
  }


  public void HomeElevator() {
    if (lowerLimitSwitch.isPressed()) {
      runElevatorManualVoltage(0);
      primaryencoder.setPosition(Elevator.kHomingEncoderLocation);
      SmartDashboard.putBoolean("Homed", true);
      return;
    }
    if (!lowerLimitSwitch.isPressed() && getCurrentPosition() <= Elevator.kEncoderNearZero
        && primary.getOutputCurrent() > Elevator.kHomingEmergencyCurrent) {
      primaryencoder.setPosition(Elevator.kHomingEncoderLocation);
      SmartDashboard.putBoolean("EMERGENCY HOMED!!!", true);
      runElevatorManualVoltage(0);
      return;
    } else {
      runElevatorManualVoltage(Elevator.kHomingVoltage);
    }
    return;
  }

  public static SUB_Elevator getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new SUB_Elevator();
    }
    return INSTANCE;
  }

  public void periodic() {
    SmartDashboard.putNumber("PrimaryEncoder", primaryencoder.getPosition());
  }
}
