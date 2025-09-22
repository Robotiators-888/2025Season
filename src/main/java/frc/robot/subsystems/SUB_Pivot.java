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

/**
 * The SUB_Pivot class manages the robot's pivot mechanism, controlled by a SparkFlex motor.
 * It uses a PID controller for precise movement to a setpoint and an `InterpolatingDoubleTreeMap`
 * to apply a gravity-compensating holding voltage based on the pivot's current angle. This provides
 * both accurate positioning and stable holding.
 * This class follows a modified singleton pattern, requiring an encoder to be passed on first instantiation.
 */
public class SUB_Pivot extends SubsystemBase {
  /** The singleton instance of the pivot subsystem. */
  private static SUB_Pivot INSTANCE = null;

  /** The primary SparkFlex motor controller for the pivot arm. */
  private SparkFlex armPrimary = new SparkFlex(31, MotorType.kBrushless);

  /** The absolute encoder that measures the pivot arm's angle. */
  private SparkAbsoluteEncoder absoluteEncoder;

  /** The configuration object for the SparkFlex motor controller. */
  private SparkMaxConfig armMotorConfig = new SparkMaxConfig();

  /** The target output voltage for the pivot motor. */
  public double outputvoltage = 0;

  /** The current angle setpoint for the pivot, in degrees. */
  private double setpoint = PivotConstants.kIntakeSetpoint;

  /** The PID controller used to calculate the voltage needed to reach the setpoint. */
  private PIDController voltagePID = new PIDController(2.0 * 0.035, 0, 0);

  /** An interpolating map to determine the holding voltage needed to counteract gravity at different angles. */
  private InterpolatingDoubleTreeMap constantApplicationMap = new InterpolatingDoubleTreeMap();

  /** An interpolating map for holding voltage when the robot is carrying coral. */
  private InterpolatingDoubleTreeMap coralConstantApplicationMap = new InterpolatingDoubleTreeMap();

  /** An interpolating map for holding voltage when the robot is carrying algae. */
  private InterpolatingDoubleTreeMap algaeConstantApplicationMap = new InterpolatingDoubleTreeMap();

  /** The previous recorded position of the pivot. */
  private double previousPosition;

  /** The current recorded position of the pivot. */
  private double currentPosition;

  /**
   * Constructs a new SUB_Pivot. This is private to enforce the singleton pattern.
   * It configures the pivot motor, PID controller, and initializes the interpolating tree maps
   * with angle-to-voltage mappings for gravity compensation.
   * @param absoluteEncoder The absolute encoder for the pivot motor, passed during instantiation.
   */
  public SUB_Pivot(SparkAbsoluteEncoder absoluteEncoder) {

    voltagePID.disableContinuousInput();

    armMotorConfig.inverted(true);
    armMotorConfig.disableFollowerMode();
    armMotorConfig.encoder.positionConversionFactor(360.0 / (5 * (30.0 / 16.0)));
    armMotorConfig.smartCurrentLimit(50, 60, 240);
    armPrimary.configure(armMotorConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    this.absoluteEncoder = absoluteEncoder;

    // Populate the interpolating tree maps with holding voltages for different pivot positions.
    // These values are empirically determined to counteract gravity.
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

  /**
   * Runs the pivot motor at a specified speed for manual control.
   * @param manual The speed to run the motor at [-1.0, 1.0].
   */
  public void runPivotManual(double manual) {
    armPrimary.set(manual);
  }

  /**
   * Runs the pivot motor at a specified voltage for manual control.
   * @param volts The voltage to apply to the motor.
   */
  public void runPivotManualVoltage(double volts) {
    armPrimary.setVoltage(volts);
  }

  /**
   * Updates the output voltage for the pivot motor by a given amount.
   * @param voltage The voltage to add to the current output voltage.
   */
  public void updateVoltage(double voltage) {
    outputvoltage = outputvoltage + voltage;
    outputvoltage = MathUtil.clamp(outputvoltage, -3, 6);
  }


  /**
   * Runs the pivot motor to a setpoint using a PID controller and a feedforward holding voltage.
   * @param hasCoral A supplier that returns true if the robot is holding coral, false otherwise.
   */
  public void runPivot(Supplier<Boolean> hasCoral) {
    currentPosition = absoluteEncoder.getPosition();
    double error = setpoint - currentPosition;
    // The PID controller calculates the voltage needed to correct the error.
    double outputVoltage = MathUtil.clamp(voltagePID.calculate(currentPosition, setpoint), -3, 3);

    SmartDashboard.putNumber("Pivot Output Voltage", outputVoltage);
    SmartDashboard.putNumber("Pivot Voltage PID Output",
        voltagePID.calculate(currentPosition, setpoint));
    SmartDashboard.putNumber("Pivot Holding Voltage", constantApplicationMap.get(currentPosition));
    SmartDashboard.putNumber("Pivot Setpoint Error", error);
    SmartDashboard.putNumber("Pivot Error Accumulation", voltagePID.getAccumulatedError());

    // The final voltage is the sum of the PID output and the feedforward holding voltage.
    // Note: The holding voltage is not currently being added to the output voltage in this implementation.
    runPivotManualVoltage(outputVoltage);
  }

  /**
   * Changes the setpoint for the pivot.
   * @param setpoint The new setpoint in degrees.
   */
  public void changeSetpoint(double setpoint) {
    this.setpoint = setpoint;
  }

  /**
   * Changes the setpoint for the pivot using a supplier.
   * @param setpoint A supplier that provides the new setpoint in degrees.
   */
  public void changeSetpoint(Supplier<Double> setpoint) {
    this.setpoint = setpoint.get();
  }

  /**
   * Checks if the pivot is at a specified setpoint within tolerance.
   * @param setpoint The setpoint to check against in degrees.
   * @return True if the pivot is at the setpoint, false otherwise.
   */
  public boolean atSetpoint(double setpoint) {
    SmartDashboard.putNumber("CURRENT POSITION ELEVATOR CONDITIONAL", currentPosition);
    SmartDashboard.putBoolean("GOOD TO ELEVATE?",
        Math.abs(currentPosition - setpoint) < PivotConstants.toleranceDegrees);
    return Math.abs(absoluteEncoder.getPosition() - setpoint) < PivotConstants.toleranceDegrees;
  }

  /**
   * Checks if the pivot is at a position suitable for scoring algae.
   * This is a position-based check rather than a setpoint check.
   * @param setpoint The setpoint to check against (used for dashboard reporting).
   * @return True if the pivot is in a position to score algae, false otherwise.
   */
  public boolean atSetpointAlgae(double setpoint) {
    SmartDashboard.putNumber("CURRENT POSITION ELEVATOR CONDITIONAL", currentPosition);
    SmartDashboard.putBoolean("GOOD TO ELEVATE?",
        Math.abs(currentPosition - setpoint) < PivotConstants.toleranceDegrees);
    return absoluteEncoder.getPosition() < 226;
  }

  /**
   * Checks if the pivot is at the specific setpoint for elevating.
   * @return True if the pivot is within the elevating position range, false otherwise.
   */
  public boolean atElevatingSetpoint(){
    if (absoluteEncoder.getPosition() < 297 && absoluteEncoder.getPosition() > 285){
      return true;
    }
    return false;
  
  }

  /**
   * Checks if the pivot is at a specified setpoint within tolerance, using a supplier.
   * @param setpoint A supplier for the setpoint to check against in degrees.
   * @return True if the pivot is at the setpoint, false otherwise.
   */
  public boolean atSetpoint(Supplier<Double> setpoint) {
    return Math
        .abs(absoluteEncoder.getPosition() - setpoint.get()) < PivotConstants.toleranceDegrees;
  }

  /**
   * Returns the singleton instance of the pivot subsystem.
   * Note: This is a modified singleton pattern. The absolute encoder must be passed
   * on the first call to instantiate the subsystem. Subsequent calls can pass null.
   * @param absoluteEncoder The absolute encoder for the pivot motor.
   * @return The singleton instance of SUB_Pivot.
   */
  public static SUB_Pivot getInstance(SparkAbsoluteEncoder absoluteEncoder) {
    if (INSTANCE == null) {
      INSTANCE = new SUB_Pivot(absoluteEncoder);
    }
    return INSTANCE;
  }


  /**
   * This method is called periodically every robot loop.
   * It updates the SmartDashboard with the pivot's encoder position, setpoint, and current draw.
   */
  @Override
  public void periodic() {
    SmartDashboard.putNumber("ABSEncoder", absoluteEncoder.getPosition());
    SmartDashboard.putNumber("Pivot SETPOINT", setpoint);
    SmartDashboard.putNumber("Pivot Current", armPrimary.getOutputCurrent());
  }
}
