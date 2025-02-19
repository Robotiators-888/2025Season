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
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;

public class SUB_Pivot extends SubsystemBase {
  private static SUB_Pivot INSTANCE = null;

  private SparkMax armPrimary = new SparkMax(31, MotorType.kBrushless);
  private SparkAbsoluteEncoder absoluteEncoder;
  private SparkMaxConfig armMotorConfig = new SparkMaxConfig();
  public double outputvoltage = 0;

  private double setpoint = 0;// TOOD: Change
  private PIDController voltagePID = new PIDController(0, 0, 0); // TODO: Change constants

  private InterpolatingDoubleTreeMap constantApplicationMap = new InterpolatingDoubleTreeMap();
  private InterpolatingDoubleTreeMap coralConstantApplicationMap = new InterpolatingDoubleTreeMap();
  private InterpolatingDoubleTreeMap algaeConstantApplicationMap = new InterpolatingDoubleTreeMap();


  public SUB_Pivot(SparkAbsoluteEncoder absoluteEncoder) {
    armMotorConfig.inverted(true);
    armMotorConfig.disableFollowerMode();
    armPrimary.configure(armMotorConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
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

  public void runPivot(Supplier<Boolean> hasCoral, Supplier<Boolean> hasAlgae) {
    double error = setpoint - absoluteEncoder.getPosition();

    double outputVoltage = voltagePID.calculate(error);

    if (hasCoral.get()) {
      outputVoltage += coralConstantApplicationMap.get(absoluteEncoder.getPosition());
    } else if (hasAlgae.get()) {
      outputVoltage += algaeConstantApplicationMap.get(absoluteEncoder.getPosition());
    } else {
      outputVoltage += constantApplicationMap.get(absoluteEncoder.getPosition());
    }

    SmartDashboard.putNumber("Pivot Output Voltage", outputVoltage);

    runPivotManualVoltage(outputVoltage);
  }

  public void changeSetpoint(double setpoint) {
    this.setpoint = setpoint;
  }

  public void changeSetpoint(Supplier<Double> setpoint) {
    this.setpoint = setpoint.get();
  }

  public boolean atSetpoint(double setpointRadians) {
    return Math
        .abs(absoluteEncoder.getPosition() - setpointRadians) < PivotConstants.toleranceRadians;
  }

  public boolean atSetpoint(Supplier<Double> setpointRadians) {
    return Math.abs(
        absoluteEncoder.getPosition() - setpointRadians.get()) < PivotConstants.toleranceRadians;
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
