// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
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

  private SparkFlex armPrimary = new SparkFlex(31, MotorType.kBrushless);
  private SparkAbsoluteEncoder absoluteEncoder;
  private SparkMaxConfig armMotorConfig = new SparkMaxConfig();
  public double outputvoltage = 0;
  // public double outputvoltage2 = 0;

  private double setpoint = PivotConstants.kIntakeSetpoint;// TODO: Change
  private PIDController voltagePID = new PIDController(0.035, 0, 0.0035); // TODO: Change

  private InterpolatingDoubleTreeMap constantApplicationMap = new InterpolatingDoubleTreeMap();
  private InterpolatingDoubleTreeMap coralConstantApplicationMap = new InterpolatingDoubleTreeMap();
  private InterpolatingDoubleTreeMap algaeConstantApplicationMap = new InterpolatingDoubleTreeMap();

  private double previousPosition;
  private double currentPosition;

  public SUB_Pivot(SparkAbsoluteEncoder absoluteEncoder) {

    voltagePID.disableContinuousInput();

    armMotorConfig.inverted(true);
    armMotorConfig.disableFollowerMode();
    armMotorConfig.encoder.positionConversionFactor(360.0 / (5 * (30.0 / 16.0)));
    armMotorConfig.smartCurrentLimit(50, 60, 240);
    armPrimary.configure(armMotorConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    this.absoluteEncoder = absoluteEncoder;

    constantApplicationMap.put(1.0, 0.425);
    constantApplicationMap.put(156.0, 0.425);
    constantApplicationMap.put(185.0, 0.6);
    constantApplicationMap.put(226.5, 0.475);
    constantApplicationMap.put(264.0, 0.2);
    constantApplicationMap.put(285.0, 0.0); // 0.791666667
    constantApplicationMap.put(304.0, -0.225);
    constantApplicationMap.put(324.0, -0.35);
    constantApplicationMap.put(327.0, -0.4);
    constantApplicationMap.put(338.0, -0.475);
    constantApplicationMap.put(359.0, -0.475);

    coralConstantApplicationMap.put(-1.0, 0.6);
    coralConstantApplicationMap.put(160.0, 0.6);
    coralConstantApplicationMap.put(195.0, 0.75);
    coralConstantApplicationMap.put(225.8, 0.725);
    coralConstantApplicationMap.put(252.0, 0.5);
    coralConstantApplicationMap.put(320.0, -0.425);
    constantApplicationMap.put(359.0, -0.425);

    previousPosition = absoluteEncoder.getPosition();
    currentPosition = absoluteEncoder.getPosition();
  }

  public void runPivotManual(double manual) {
    armPrimary.set(manual);
  }

  public void runPivotManualVoltage(double volts) {
    armPrimary.setVoltage(volts);
  }


  // public void runPivotHoldingVoltage(Supplier<Boolean> hasCoral) {
  // if (hasCoral.get()) {
  // armPrimary.setVoltage(coralConstantApplicationMap.get(absoluteEncoder.getPosition()));
  // } else {
  // armPrimary.setVoltage(constantApplicationMap.get(absoluteEncoder.getPosition()));
  // }
  // }

  public void updateVoltage(double voltage) {
    outputvoltage = outputvoltage + voltage;
    outputvoltage = MathUtil.clamp(outputvoltage, -3, 6);
  }



  public void runPivot(Supplier<Boolean> hasCoral) {
    currentPosition = absoluteEncoder.getPosition();
    double error = setpoint - currentPosition;
    double outputVoltage = MathUtil.clamp(voltagePID.calculate(currentPosition, setpoint), -3, 3);
    // if (setpoint == PivotConstants.kIntakeSetpoint && absoluteEncoder.getPosition() > 334.5){
    // outputVoltage -= 0.2;
    // SmartDashboard.putBoolean("Has Triggered", true);
    // }

    // if (setpoint == PivotConstants.kIntakeSetpoint && currentPosition > 333) {
    // outputVoltage -= 0.2;
    // // }
    // if (hasCoral.get()) {
    // SmartDashboard.putBoolean("coral constants?", true);
    // outputVoltage += coralConstantApplicationMap.get(currentPosition);
    // } else {
    // outputVoltage += constantApplicationMap.get(currentPosition);
    // }

    // if (absoluteEncoder.getPosition() < PivotConstants.kUpperBoundStuckPoint
    // && absoluteEncoder.getPosition() > PivotConstants.kLowerBoundStuckPoint) {
    // outputVoltage -= 0.2;
    // }

    SmartDashboard.putNumber("Pivot Output Voltage", outputVoltage);
    SmartDashboard.putNumber("Pivot Voltage PID Output",
        voltagePID.calculate(currentPosition, setpoint));
    SmartDashboard.putNumber("Pivot Holding Voltage", constantApplicationMap.get(currentPosition));
    SmartDashboard.putNumber("Pivot Setpoint Error", error);
    SmartDashboard.putNumber("Pivot Error Accumulation", voltagePID.getAccumulatedError());

    runPivotManualVoltage(outputVoltage);
  }

  public void changeSetpoint(double setpoint) {
    this.setpoint = setpoint;
  }

  public void changeSetpoint(Supplier<Double> setpoint) {
    this.setpoint = setpoint.get();
  }

  public boolean atSetpoint(double setpoint) {
    SmartDashboard.putNumber("CURRENT POSITION ELEVATOR CONDITIONAL", currentPosition);
    SmartDashboard.putNumber("cdl", Math.abs(currentPosition - setpoint));
    SmartDashboard.putBoolean("GOOD TO ELEVATE?",
        Math.abs(currentPosition - setpoint) < PivotConstants.toleranceDegrees);
    return Math.abs(absoluteEncoder.getPosition() - setpoint) < PivotConstants.toleranceDegrees;
  }

  public boolean atSetpoint(Supplier<Double> setpoint) {
    return Math
        .abs(absoluteEncoder.getPosition() - setpoint.get()) < PivotConstants.toleranceDegrees;
  }

  // public void changeVoltage(double voltage) {
  // outputvoltage2 += voltage;
  // runPivotManualVoltage(outputvoltage2);
  // SmartDashboard.putNumber("Pivot Output Voltage", outputvoltage2);
  // }

  public static SUB_Pivot getInstance(SparkAbsoluteEncoder absoluteEncoder) {
    if (INSTANCE == null) {
      INSTANCE = new SUB_Pivot(absoluteEncoder);
    }
    return INSTANCE;
  }


  public void periodic() {
    SmartDashboard.putNumber("ABSEncoder", absoluteEncoder.getPosition());
    SmartDashboard.putNumber("ABSEncoder Position", currentPosition);
    SmartDashboard.putNumber("Pivot SETPOINT", setpoint);

    // if (Math.abs(currentPosition - previousPosition) > 50) {
    // currentPosition = previousPosition;
    // SmartDashboard.putBoolean("SPIKED??", true);
    // } else {
    // previousPosition = currentPosition;
    // }
    SmartDashboard.putNumber("Pivot Current", armPrimary.getOutputCurrent());

    SmartDashboard.putNumber("ABSEncoder Position", currentPosition);
  }
}
