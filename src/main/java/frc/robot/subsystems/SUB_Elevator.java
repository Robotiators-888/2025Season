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


/**
 * The SUB_Elevator class represents the elevator subsystem of the robot.
 * It manages two SparkMax motors (one primary, one follower) that control the elevator's vertical movement.
 * It uses an encoder for position feedback and a limit switch for homing. The control logic is a
 * state machine that applies different voltages based on the error from the setpoint and whether
 * the robot is holding a game piece.
 * This class follows a singleton pattern.
 */
public class SUB_Elevator extends SubsystemBase {
  /** The singleton instance of the elevator subsystem. */
  private static SUB_Elevator INSTANCE = null;

  /** The active position setpoint for the elevator, in meters. */
  private static double activesetpoint = 0;

  /** The primary SparkMax motor controller for the elevator. */
  private SparkMax primary = new SparkMax(35, MotorType.kBrushless);

  /** The secondary SparkMax motor controller, configured to follow the primary. */
  private SparkMax secondary = new SparkMax(36, MotorType.kBrushless);

  /** The configuration object for the SparkMax motor controllers. */
  private SparkMaxConfig config = new SparkMaxConfig();

  /** The encoder attached to the primary motor for position feedback. */
  private RelativeEncoder primaryencoder = primary.getEncoder();

  /** The lower limit switch, used for homing the elevator to a known zero position. */
  private SparkLimitSwitch lowerLimitSwitch = primary.getReverseLimitSwitch();

  /** The instance of the roller subsystem, used to check for game pieces. */
  public static SUB_Roller roller = SUB_Roller.getInstance();


  /**
   * Constructs a new SUB_Elevator.
   * This constructor is private to enforce the singleton pattern.
   * It configures the primary and secondary SparkMax motors, setting one to follow the other
   * and configuring inversion, idle mode, and encoder conversion factors.
   */
  private SUB_Elevator() {
    config.follow(primary);
    config.inverted(true);
    config.idleMode(IdleMode.kBrake);
    secondary.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    config.inverted(false);
    config.disableFollowerMode();
    config.encoder.positionConversionFactor((0.2 * Units.inchesToMeters(1.92 * Math.PI)));
    config.encoder.velocityConversionFactor((0.2 * Units.inchesToMeters(1.92 * Math.PI)) / 60);
    primary.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /**
   * Runs the elevator motors at a specified voltage for manual control.
   * @param volts The voltage to apply to the motors.
   */
  public void runElevatorManualVoltage(double volts) {
    primary.setVoltage(volts);
  }

  /**
   * Resets the elevator's primary encoder position to zero.
   */
  public void zeroEncoder() {
    primaryencoder.setPosition(0);
  }

  /**
   * Returns the active setpoint for the elevator's position.
   * @return The active setpoint in meters.
   */
  public double getActiveSetpoint(){
    return activesetpoint;
  }

  /**
   * Returns the primary encoder for the elevator.
   * @return The primary RelativeEncoder object.
   */
  public RelativeEncoder getPrimaryEncoder(){
    return primaryencoder;
  }
  
  /**
   * Returns the primary SparkMax motor controller for the elevator.
   * @return The primary SparkMax object.
   */
  public SparkMax getPrimary(){
    return primary;
  }


  /**
   * Runs the elevator with a state-based logic to move to and hold a setpoint.
   * This method uses different voltages for moving up, down, and holding,
   * depending on the error from the setpoint and whether it has a game piece.
   * @param pivotSafe A supplier that returns true if the pivot is in a safe position for elevator movement.
   */
  public void runElevator(Supplier<Boolean> pivotSafe) {
    if (getCurrentPosition() >= Elevator.kResetHomingThreshold) {
      SmartDashboard.putBoolean("EMERGENCY HOMED!!!", false);
      SmartDashboard.putBoolean("Homed", false);
    }
    SmartDashboard.putNumber("Elevator Output Voltage", primary.getAppliedOutput() * primary.getBusVoltage());
    SmartDashboard.putNumber("EncoderPos", primaryencoder.getPosition());

    // If the pivot is not in a safe position and we are not at the setpoint, hold the elevator.
    if (!pivotSafe.get() && !this.atSetpoint()){
      SmartDashboard.putBoolean("Elevator is Safe", false);
      // Apply holding voltage based on game piece presence.
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

    SmartDashboard.putBoolean("Elevator is Safe", true);
    // If the setpoint is at the bottom and we are close, initiate homing.
    if (activesetpoint <= 0 && getCurrentPosition() <= Elevator.kSlowDownThreshold) {
      HomeElevator();
      return;
    }

    // If at the setpoint, apply holding voltage.
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

    // If below the setpoint, move up with tiered voltage based on error.
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

    // If above the setpoint, move down with tiered voltage based on error.
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

  /**
   * Runs the elevator with a state-based logic specifically for algae scoring.
   * This method is largely a duplicate of runElevator and could be refactored.
   * @param pivotSafe A supplier that returns true if the pivot is in a safe position for elevator movement.
   */
  public void runElevatorAlgae(Supplier<Boolean> pivotSafe) {
    if (getCurrentPosition() >= Elevator.kResetHomingThreshold) {
      SmartDashboard.putBoolean("EMERGENCY HOMED!!!", false);
      SmartDashboard.putBoolean("Homed", false);
    }
    SmartDashboard.putNumber("Elevator Output Voltage", primary.getAppliedOutput() * primary.getBusVoltage());
    SmartDashboard.putNumber("EncoderPos", primaryencoder.getPosition());
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

    SmartDashboard.putBoolean("Elevator is Safe", true);
    if (activesetpoint <= 0 && getCurrentPosition() <= Elevator.kSlowDownThreshold) {
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


  /**
   * Checks if the elevator is at a specified setpoint within a given tolerance.
   * @param setpoint The setpoint to check against in meters.
   * @return True if the elevator is at the setpoint, false otherwise.
   */
  public boolean atSetpoint(double setpoint) {
    return Math.abs(primaryencoder.getPosition() - setpoint) < Elevator.kTolerance;
  }

  /**
   * Checks if the elevator is at the currently active setpoint.
   * @return True if the elevator is at the active setpoint, false otherwise.
   */
  public boolean atSetpoint() {
    return atSetpoint(activesetpoint);
  }

  /**
   * Returns the current position of the elevator.
   * @return The current position of the elevator in meters.
   */
  public double getCurrentPosition() {
    return primaryencoder.getPosition();
  }

  /**
   * Changes the active setpoint for the elevator.
   * @param setpoint The new setpoint in meters.
   */
  public void ChangeSetpoint(double setpoint) {
    activesetpoint = setpoint;
    SmartDashboard.putNumber("Elevator Target Setpoint", setpoint);
  }


  /**
   * Homes the elevator by moving it down until the lower limit switch is pressed.
   * If the limit switch is pressed, the encoder is reset.
   * It also includes an emergency homing feature that triggers if the elevator is near
   * the bottom and drawing high current, which might indicate a stall.
   */
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

  /**
   * Returns the singleton instance of the elevator subsystem.
   * @return The singleton instance of the SUB_Elevator.
   */
  public static SUB_Elevator getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new SUB_Elevator();
    }
    return INSTANCE;
  }

  /**
   * This method is called periodically every robot loop.
   * It updates the SmartDashboard with the elevator's encoder position.
   */
  @Override
  public void periodic() {
    SmartDashboard.putNumber("PrimaryEncoder", primaryencoder.getPosition());
  }
}
