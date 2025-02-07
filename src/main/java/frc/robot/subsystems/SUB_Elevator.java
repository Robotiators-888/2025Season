// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Elevator;


public class SUB_Elevator extends SubsystemBase {
  private static SUB_Elevator INSTANCE = null;
  private static boolean hasHomed = Elevator.kStartingHoming;
  private static SparkMax primary = new SparkMax(35, MotorType.kBrushless);
  private static SparkMax secondary = new SparkMax(36, MotorType.kBrushless);
  private static SparkMaxConfig config = new SparkMaxConfig();
  private static RelativeEncoder primaryencoder = primary.getEncoder();
  private static SparkLimitSwitch lowerLimitSwitch = primary.getForwardLimitSwitch();
  private static SparkClosedLoopController primaryPID = primary.getClosedLoopController();
  private static ElevatorFeedforward feedforward =
      new ElevatorFeedforward(0, Elevator.kG, Elevator.kV);
  private final TrapezoidProfile profile =
      new TrapezoidProfile(new TrapezoidProfile.Constraints(1.75, 0.75));
  private TrapezoidProfile.State goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();
  private TrapezoidProfile.State currentState = new TrapezoidProfile.State();

  private SUB_Elevator() {
    config.follow(primary);
    config.inverted(true);
    secondary.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    config.inverted(false);
    config.disableFollowerMode();
    config.encoder.positionConversionFactor((0.2 * Units.inchesToMeters(1.92 * Math.PI)) / 60);
    config.encoder.velocityConversionFactor((0.2 * Units.inchesToMeters(1.92 * Math.PI)) / 60);
    config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(Elevator.kP, Elevator.kI,
        Elevator.kD);
    primary.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    primaryPID.setReference(0, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  public void RunElevatorManual(double manual) {
    primary.set(manual);
  }

  public void RunElevator() {
    currentState =
        new TrapezoidProfile.State(primaryencoder.getPosition(), primaryencoder.getVelocity());
    setpoint = profile.calculate(Elevator.kTimeStep, currentState, goal);
    double ff = feedforward.calculate(setpoint.velocity);
    primaryPID.setReference(0.0, ControlType.kPosition, ClosedLoopSlot.kSlot0, ff,
        ArbFFUnits.kVoltage);
  }

  public void ChangeSetpoint(double setpoint) {
    goal = new TrapezoidProfile.State(setpoint, 0);
    SmartDashboard.putNumber("Elevator Target Setpoint", setpoint);
  }


  public void HomeElevator() {
    if (lowerLimitSwitch.isPressed()) {
      RunElevatorManual(0);
      primaryencoder.setPosition(Elevator.kHomingEncoderLocation);
      hasHomed = true;
      SmartDashboard.putBoolean("Homed", hasHomed);
      return;
    }
    if (!lowerLimitSwitch.isPressed() && primaryencoder.getPosition() < Elevator.kEncoderNearZero
        && primary.getOutputCurrent() > Elevator.kHomingEmergencyCurrent) {
      primaryencoder.setPosition(Elevator.kHomingEncoderLocation);
      SmartDashboard.putBoolean("EMERGENCY HOMED!!!", true);
      RunElevatorManual(0);
      return;
    } else {
      RunElevatorManual(Elevator.kHomingSpeed);
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
