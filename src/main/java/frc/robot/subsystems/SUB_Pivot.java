// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import java.util.function.Supplier;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;

public class SUB_Pivot extends SubsystemBase {
  private static SUB_Pivot INSTANCE = null;

  private SparkMax armPrimary = new SparkMax(31, MotorType.kBrushless);
  private SparkAbsoluteEncoder absoluteEncoder;
  private SparkMaxConfig armMotorConfig = new SparkMaxConfig();

  private TrapezoidProfile profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(2*Math.PI, 2*Math.PI)); // in rads/sec and rads/sec^2
  private TrapezoidProfile.State goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();
  private TrapezoidProfile.State currentState = new TrapezoidProfile.State();
  public double outputvoltage = 0;

  private ProfiledPIDController voltagePID =
      new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(2*Math.PI, 2*Math.PI)); // TODO: Change constants



  public SUB_Pivot(SparkAbsoluteEncoder absoluteEncoder) {
    armMotorConfig.inverted(true);
    armMotorConfig.disableFollowerMode();
    armPrimary.configure(armMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    this.absoluteEncoder = absoluteEncoder;

  }

  public void runPivotManual(double manual) {
    armPrimary.set(manual);
  }

  public void runPivotManualVoltage(double volts) {
    armPrimary.setVoltage(volts);
  }

  public void updateVoltage(double voltage) {
    outputvoltage = outputvoltage + voltage;
    outputvoltage = MathUtil.clamp(outputvoltage, -3, 6);
  }

  public void runPivot(Supplier<Boolean> hasCoral) {
    currentState =
        new TrapezoidProfile.State(absoluteEncoder.getPosition(), absoluteEncoder.getVelocity());
    setpoint = profile.calculate(0.02, currentState, goal);

    double ff = PivotConstants.noCoralArmFeedforward.calculate(setpoint.position, setpoint.velocity);
    if (hasCoral.get()){
      ff = PivotConstants.coralArmFeedforward.calculate(setpoint.position, setpoint.velocity);
    }

    armPrimary.setVoltage(voltagePID.calculate(absoluteEncoder.getPosition(), setpoint.position) + ff);
  }

  public void changeSetpoint(double setpoint) {
    goal = new TrapezoidProfile.State(setpoint, 0);
    SmartDashboard.putNumber("Arm Target Setpoint", setpoint);
  }

  public void changeSetpoint(Supplier<Double> setpoint) {
    goal = new TrapezoidProfile.State(setpoint.get(), 0);
  }

  public boolean atSetpoint(double setpointRadians) {
    return Math.abs(absoluteEncoder.getPosition()-setpointRadians) < PivotConstants.toleranceRadians;
  }

  public boolean atSetpoint(Supplier<Double> setpointRadians) {
    return Math.abs(absoluteEncoder.getPosition()-setpointRadians.get()) < PivotConstants.toleranceRadians;
  }

  public static SUB_Pivot getInstance(SparkAbsoluteEncoder absoluteEncoder) {
    if (INSTANCE == null) {
      INSTANCE = new SUB_Pivot(absoluteEncoder);
    }
    return INSTANCE;
  }

  public void periodic() {
    SmartDashboard.putNumber("ABSEncoder", absoluteEncoder.getPosition());
  }
}
