package frc.robot.subsystems;

import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * The MAXSwerveModule class represents a single REV MAXSwerve module.
 * It encapsulates the driving and turning motors, encoders, and PID controllers for one module,
 * providing an interface to control and query its state.
 */
public class MAXSwerveModule {
  /** The SparkFlex motor controller for driving the module. */
  private final SparkFlex m_drivingSparkFlex;

  /** The SparkMax motor controller for turning the module. */
  private final SparkMax m_turningSparkMax;

  /** The encoder for the driving motor, measures linear distance. */
  private final RelativeEncoder m_drivingEncoder;

  /** The encoder for the turning motor, measures absolute angle. */
  private final AbsoluteEncoder m_turningEncoder;

  /** The closed-loop PID controller for the driving motor. */
  private final SparkClosedLoopController m_drivingClosedLoopController;

  /** The closed-loop PID controller for the turning motor. */
  private final SparkClosedLoopController m_turningClosedLoopController;

  /** The angular offset of the module from the chassis, in radians. */
  private double m_chassisAngularOffset = 0;

  /** The desired state of the module (speed and angle). */
  // Constructors can also be used as so (nestled just like functions)
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor, encoder, and PID
   * controller. This configuration is specific to the REV MAXSwerve Module built with NEOs, SPARKS
   * MAX, and a Through Bore Encoder.
   *
   * @param drivingCANId The CAN ID of the driving motor.
   * @param turningCANId The CAN ID of the turning motor.
   * @param chassisAngularOffset The angular offset of the module from the chassis.
   */
  public MAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
    m_drivingSparkFlex = new SparkFlex(drivingCANId, SparkLowLevel.MotorType.kBrushless);
    m_turningSparkMax = new SparkMax(turningCANId, SparkLowLevel.MotorType.kBrushless);
    m_drivingEncoder = m_drivingSparkFlex.getEncoder();
    m_turningEncoder = m_turningSparkMax.getAbsoluteEncoder();
    m_drivingClosedLoopController = m_drivingSparkFlex.getClosedLoopController();
    m_turningClosedLoopController = m_turningSparkMax.getClosedLoopController();

    // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
    m_drivingSparkFlex.configure(SwerveModuleConfigs.MAXSwerveModule.drivingConfig,
        ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_turningSparkMax.configure(SwerveModuleConfigs.MAXSwerveModule.turningConfig,
        ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


    m_chassisAngularOffset = chassisAngularOffset;
    m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
    m_drivingEncoder.setPosition(0);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module as a SwerveModuleState object.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(m_drivingEncoder.getVelocity(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module as a SwerveModulePosition object.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(m_drivingEncoder.getPosition(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Sets the position of the driving encoder.
   *
   * @param posMeters The position in meters.
   */
  public void setPosition(double posMeters) {
    m_drivingEncoder.setPosition(posMeters);
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState The desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle =
        desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    @SuppressWarnings("deprecation")
    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
        new Rotation2d(m_turningEncoder.getPosition()));

    // Command driving and turning SPARKS MAX towards their respective setpoints.
    m_drivingClosedLoopController.setReference(optimizedDesiredState.speedMetersPerSecond,
        SparkFlex.ControlType.kVelocity);
    m_turningClosedLoopController.setReference(optimizedDesiredState.angle.getRadians(),
        SparkMax.ControlType.kPosition);

    m_desiredState = desiredState;
  }

  /**
   * Resets the driving encoder to a position of 0.
   */
  public void resetEncoders() {
    m_drivingEncoder.setPosition(0);
  }

  /**
   * Returns the velocity of the driving motor.
   *
   * @return The velocity of the driving motor in meters per second.
   */
  public double getVelocityDrive() {
    return m_drivingEncoder.getVelocity();
  }

  /**
   * Returns the velocity of the turning motor.
   *
   * @return The velocity of the turning motor in radians per second.
   */
  public double getVelocitySteer() {
    return m_turningEncoder.getVelocity();
  }
}
