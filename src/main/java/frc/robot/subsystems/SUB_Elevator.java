// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Elevator;


public class SUB_Elevator extends SubsystemBase {
  private static SUB_Elevator INSTANCE = null;
  private boolean hasHomed = Elevator.kStartingHoming;
  private SparkMax primary = new SparkMax(35, MotorType.kBrushless);
  private SparkMax secondary = new SparkMax(36, MotorType.kBrushless);
  private SparkMaxConfig config = new SparkMaxConfig();
  private RelativeEncoder primaryencoder = primary.getEncoder();
  private SparkLimitSwitch lowerLimitSwitch = primary.getForwardLimitSwitch();
  private SparkClosedLoopController primaryPID = primary.getClosedLoopController();
  private final TrapezoidProfile profile =
      new TrapezoidProfile(new TrapezoidProfile.Constraints(1.0, 0.5));
  public TrapezoidProfile.State goal = new TrapezoidProfile.State();
  public TrapezoidProfile.State setpoint = new TrapezoidProfile.State();
  public TrapezoidProfile.State currentState = new TrapezoidProfile.State();
  public double outputvoltage = 0;

  public InterpolatingDoubleTreeMap encoderToKg = new InterpolatingDoubleTreeMap();


  private SUB_Elevator() {

    config.follow(primary);
    config.inverted(false);
    config.idleMode(IdleMode.kBrake);
    secondary.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    config.inverted(true);
    config.disableFollowerMode();
    config.encoder.positionConversionFactor((0.2 * Units.inchesToMeters(1.92 * Math.PI)));
    config.encoder.velocityConversionFactor((0.2 * Units.inchesToMeters(1.92 * Math.PI)) / 60);
    config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(Elevator.kP, Elevator.kI,
        Elevator.kD);
    primary.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    currentState =
        new TrapezoidProfile.State(primaryencoder.getPosition(), primaryencoder.getVelocity());

    encoderToKg.put(Double.MIN_VALUE, 0.3);
    encoderToKg.put(0., 0.3);
    encoderToKg.put(0.183, 0.4);
    encoderToKg.put(0.2517, 0.5);
    encoderToKg.put(Double.MAX_VALUE, 0.5);
  }

  public void runElevatorManual(double manual) {
    primary.set(manual);
  }

  public void runElevatorManualVoltage(double volts) {
    primary.setVoltage(volts);
  }

  public void updateVoltage(double voltage) {
    outputvoltage = outputvoltage + voltage;
    outputvoltage = MathUtil.clamp(outputvoltage, -0.5, 6);
  }

  public void zeroEncoder() {
    primaryencoder.setPosition(0);
  }

  public void runElevator() {
    if(goal.position - .09 < 0){
      runElevatorManualVoltage(0);
      return;
    }
    
    if(Math.abs(goal.position - primaryencoder.getPosition())<.01){
      runElevatorManualVoltage(.6);
      return;
    }

    if(goal.position > primaryencoder.getPosition()){
      if (goal.position - primaryencoder.getPosition() > .15){
        runElevatorManualVoltage(7.5);
      }
      runElevatorManualVoltage(1.5);
      return;
    }

    if(goal.position < primaryencoder.getPosition()){
      if (primaryencoder.getPosition() - goal.position < .15){
        runElevatorManualVoltage(-1.2);
      }
      runElevatorManualVoltage(.1);
      return;
    }
    runElevatorManual(0);

    // setpoint = profile.calculate(Elevator.kTimeStep, currentState, goal);
    // double feedforward =
    //     new ElevatorFeedforward(Elevator.kS, Elevator.kG, Elevator.kV).calculate(setpoint.velocity);


    // primaryPID.setReference(setpoint.position, ControlType.kPosition, ClosedLoopSlot.kSlot0, feedforward, ArbFFUnits.kVoltage);
    // SmartDashboard.putNumber("Setpoint Pos", setpoint.position);
    // SmartDashboard.putNumber("OutputVoltage", primary.getAppliedOutput() * primary.getBusVoltage());
    // SmartDashboard.putNumber("Feedforward", feedforward);
    // SmartDashboard.putNumber("Goal Pos", goal.position);
    // SmartDashboard.putNumber("Setpoint Velo", setpoint.velocity);

    // currentState =
    //     new TrapezoidProfile.State(primaryencoder.getPosition(), primaryencoder.getVelocity());
  }

  public boolean atSetpoint(double setpoint) {
    return Math.abs(primaryencoder.getPosition() - setpoint) < Elevator.kTolerance;
  }

  public double getCurrentPosition() {
    return primaryencoder.getPosition();
  }

  public void ChangeSetpoint(double setpoint) {
    goal = new TrapezoidProfile.State(setpoint, 0);
    SmartDashboard.putNumber("Elevator Target Setpoint", setpoint);
  }


  public void HomeElevator() {
    if (lowerLimitSwitch.isPressed()) {
      runElevatorManual(0);
      primaryencoder.setPosition(Elevator.kHomingEncoderLocation);
      hasHomed = true;
      SmartDashboard.putBoolean("Homed", hasHomed);
      return;
    }
    if (!lowerLimitSwitch.isPressed() && primaryencoder.getPosition() < Elevator.kEncoderNearZero
        && primary.getOutputCurrent() > Elevator.kHomingEmergencyCurrent) {
      primaryencoder.setPosition(Elevator.kHomingEncoderLocation);
      SmartDashboard.putBoolean("EMERGENCY HOMED!!!", true);
      runElevatorManual(0);
      return;
    } else {
      runElevatorManual(Elevator.kHomingSpeed);
    }
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
