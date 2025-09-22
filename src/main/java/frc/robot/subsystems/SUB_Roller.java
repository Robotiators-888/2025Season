// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Roller;


/**
 * The SUB_Roller class manages the roller mechanism, which is used for intaking and outtaking game pieces.
 * It consists of two motors (a main roller and a helper), a banner sensor to detect game pieces,
 * and encoders for velocity monitoring.
 * This class follows a singleton pattern.
 */
public class SUB_Roller extends SubsystemBase {
  /** The singleton instance of the roller subsystem. */
  private static SUB_Roller INSTANCE = null;

  /** The main roller motor controller (SparkMax). */
  private SparkMax roller = new SparkMax(Roller.kRollerCanID, MotorType.kBrushless);

  /** The helper roller motor controller (SparkMax). */
  private SparkMax helper = new SparkMax(Roller.kHelperCanID, MotorType.kBrushless);

  /** The configuration object for the SparkMax motor controllers. */
  private SparkMaxConfig config = new SparkMaxConfig();

  /** The relative encoder for the main roller motor, used for measuring RPM. */
  private RelativeEncoder encoder = roller.getEncoder();

  /** The banner sensor (DigitalInput) used to detect the presence of a game piece ("coral"). */
  private DigitalInput bannerSensor = new DigitalInput(9);

  /** The absolute encoder for the main roller motor. */
  private SparkAbsoluteEncoder absoluteEncoder = roller.getAbsoluteEncoder();

  /** A flag indicating whether the robot is currently holding algae. */
  public Boolean hasAlgae = false;

  /** A timer for timing-related intake or outtake operations. */
  private Timer timer = new Timer();


  /**
   * Constructs a new SUB_Roller.
   * This constructor is private to enforce the singleton pattern.
   * It configures the roller and helper motors with voltage compensation, inversion, and current limits.
   */
  private SUB_Roller() {
    config.voltageCompensation(12);
    config.inverted(false);
    config.smartCurrentLimit(Roller.kRollerCurrentLimit);
    config.absoluteEncoder.positionConversionFactor(360);
    config.absoluteEncoder.velocityConversionFactor(360 / 60);

    roller.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    config.smartCurrentLimit(Roller.kHelperCurrentLimit);
    config.inverted(false);
    helper.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /**
   * Starts or stops the internal timer.
   * @param start True to reset and start the timer, false to stop and reset it.
   */
  public void timerInteract(boolean start) {
    if (start) {
      timer.reset();
      timer.start();
    } else {
      timer.stop();
      timer.reset();
    }
  }

  /**
   * Sets the output speed of the main roller motor.
   * @param percent The output percentage, from -1.0 to 1.0.
   */
  public void setRollerOutput(double percent) {
    roller.set(percent);
  }

  /**
   * Sets the output speed of both the main roller and helper motors.
   * @param percent The output percentage for the main roller, from -1.0 to 1.0.
   * @param helperPercent The output percentage for the helper roller, from -1.0 to 1.0.
   */
  public void setRollerOutput(double percent,double helperPercent) {
    roller.set(percent);
    helper.set(helperPercent);
  }

  /**
   * Returns whether the banner sensor detects a game piece ("coral").
   * The sensor reads `false` when an object is present, so the value is inverted.
   * @return True if a game piece is detected, false otherwise.
   */
  public boolean getHasCoral() {
    return !bannerSensor.get();
  }

  /**
   * Returns whether the robot is flagged as having algae.
   * This is based on a software flag, not a direct sensor reading.
   * @return True if the robot has algae, false otherwise.
   */
  public boolean getHasAlgae() {
    return hasAlgae;
  }

  /**
   * Sets the software flag indicating whether the robot has algae.
   * @param hasAlgae The new value for the flag.
   */
  public void setHasAlgae(boolean hasAlgae) {
    this.hasAlgae = hasAlgae;
  }

  /**
   * Returns the absolute encoder for the main roller motor.
   * @return The SparkAbsoluteEncoder object.
   */
  public SparkAbsoluteEncoder getAbsoluteEncoder() {
    return absoluteEncoder;
  }

  /**
   * Returns the singleton instance of the roller subsystem.
   * @return The singleton instance of SUB_Roller.
   */
  public static SUB_Roller getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new SUB_Roller();
    }
    return INSTANCE;
  }

  /**
   * This method is called periodically every robot loop.
   * It updates the SmartDashboard with the state of the banner sensor, whether the robot
   * has a game piece, and telemetry from the roller motor like RPM, current, and voltage.
   */
  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Banner sensor", bannerSensor.get());
    SmartDashboard.putBoolean("Has Coral", getHasCoral());
    SmartDashboard.putNumber("Roller RPM", (encoder.getVelocity() / 60));
    SmartDashboard.putNumber("Roller Current", roller.getOutputCurrent());
    SmartDashboard.putNumber("Roller Output Voltage",
        roller.getBusVoltage() * roller.getAppliedOutput());
  }
}
