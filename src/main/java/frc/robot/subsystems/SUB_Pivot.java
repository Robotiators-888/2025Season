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
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;

public class SUB_Pivot extends SubsystemBase {
  private static SUB_Pivot INSTANCE = null;

  private SparkMax armPrimary = new SparkMax(31, MotorType.kBrushless);
  private SparkAbsoluteEncoder absoluteEncoder;
  private SparkMaxConfig armMotorConfig = new SparkMaxConfig();
  public double outputvoltage = 0;

  private double setpoint = PivotConstants.kIntakeSetpoint;// TOOD: Change
  private PIDController voltagePID = new PIDController(0.02, 0.0001, 0.002); // TODO: Change constants

  private InterpolatingDoubleTreeMap constantApplicationMap = new InterpolatingDoubleTreeMap();
  private InterpolatingDoubleTreeMap coralConstantApplicationMap = new InterpolatingDoubleTreeMap();
  private InterpolatingDoubleTreeMap algaeConstantApplicationMap = new InterpolatingDoubleTreeMap();


  public SUB_Pivot(SparkAbsoluteEncoder absoluteEncoder) {
    armMotorConfig.inverted(true);
    armMotorConfig.disableFollowerMode();
    
    armPrimary.configure(armMotorConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    this.absoluteEncoder = absoluteEncoder;

    constantApplicationMap.put(Double.MIN_VALUE, 0.425);
    constantApplicationMap.put(156.0, 0.425);
    constantApplicationMap.put(185.0, 0.6);
    constantApplicationMap.put(226.5, 0.475);
    constantApplicationMap.put(264.0, 0.2);
    constantApplicationMap.put(285.0, 0.0); // 0.791666667
    constantApplicationMap.put(304.0, -0.225);
    constantApplicationMap.put(324.0, -0.35);
    constantApplicationMap.put(327.0, -0.4);
    constantApplicationMap.put(Double.MAX_VALUE, -0.35);

    coralConstantApplicationMap.put(Double.MIN_VALUE, 0.6);
    coralConstantApplicationMap.put(160.0, 0.6);
    coralConstantApplicationMap.put(195.0, 0.75);
    coralConstantApplicationMap.put(225.8, 0.725);
    coralConstantApplicationMap.put(252.0, 0.5);
    coralConstantApplicationMap.put(320.0, -0.425);
    constantApplicationMap.put(Double.MAX_VALUE, -0.425);

  }

  public void runPivotManual(double manual) {
    armPrimary.set(manual);
  }

  public void runPivotManualVoltage(double volts) {
    armPrimary.setVoltage(volts);
  }

  public void runPivotHoldingVoltage(Supplier<Boolean> hasCoral) {
    if (hasCoral.get()){
      armPrimary.setVoltage(coralConstantApplicationMap.get(absoluteEncoder.getPosition()));
    } else {
      armPrimary.setVoltage(constantApplicationMap.get(absoluteEncoder.getPosition()));
    }
  }

  public void updateVoltage(double voltage) {
    outputvoltage = outputvoltage + voltage;
    outputvoltage = MathUtil.clamp(outputvoltage, -3, 6);
  }

  public void runPivot(Supplier<Boolean> hasCoral) {
    double error = setpoint - absoluteEncoder.getPosition();
    double outputVoltage = voltagePID.calculate(absoluteEncoder.getPosition(), setpoint);

    if (Math.abs(error) < 3){
      outputVoltage = 0;
    }

    if (hasCoral.get()) {
      outputVoltage += coralConstantApplicationMap.get(absoluteEncoder.getPosition());
    } else {
      outputVoltage += constantApplicationMap.get(absoluteEncoder.getPosition());
    }

    if (absoluteEncoder.getPosition() < PivotConstants.kUpperBoundStuckPoint && absoluteEncoder.getPosition() > PivotConstants.kLowerBoundStuckPoint) {
      outputVoltage -= 0.2;
    }

    SmartDashboard.putNumber("Pivot Output Voltage", outputVoltage);
    SmartDashboard.putNumber("Pivot Voltage PID Output", voltagePID.calculate(absoluteEncoder.getPosition(), setpoint));
    SmartDashboard.putNumber("Pivot Holding Voltage", outputVoltage - voltagePID.calculate(absoluteEncoder.getPosition(), setpoint));
    SmartDashboard.putNumber("Pivot Setpoint Error", error);
    SmartDashboard.putNumber("Pivot Error Accumulation", voltagePID.getAccumulatedError());

    runPivotManualVoltage(outputVoltage);
  }

  public void changeSetpoint(double setpoint) {
    this.setpoint = setpoint;
    voltagePID.reset(); // Reset I accumulation 
  }

  public void changeSetpoint(Supplier<Double> setpoint) {
    this.setpoint = setpoint.get();
    voltagePID.reset(); // Reset I accumulation 
  }

  public boolean atSetpoint(double setpointRadians) {
    return Math
        .abs(absoluteEncoder.getPosition() - setpointRadians) < PivotConstants.toleranceDegrees;
  }

  public boolean atSetpoint(Supplier<Double> setpointRadians) {
    return Math.abs(
        absoluteEncoder.getPosition() - setpointRadians.get()) < PivotConstants.toleranceDegrees;
  }

  public static SUB_Pivot getInstance(SparkAbsoluteEncoder absoluteEncoder) {
    if (INSTANCE == null) {
      INSTANCE = new SUB_Pivot(absoluteEncoder);
    }
    return INSTANCE;
  }


  public void periodic() {
    SmartDashboard.putNumber("ABSEncoder", absoluteEncoder.getPosition());
    SmartDashboard.putNumber("Pivot SETPOINT", setpoint);
  }
}
