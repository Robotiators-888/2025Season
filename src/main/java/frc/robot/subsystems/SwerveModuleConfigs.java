package frc.robot.subsystems;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import frc.robot.Constants.Swerve;

/**
 * The SwerveModuleConfigs class holds the configuration objects for the REV MAXSwerve modules.
 * This is a final class and is not meant to be instantiated; it serves as a centralized
 * container for the detailed configuration of the swerve module motors, which are then

 * applied in the MAXSwerveModule class.
 */
public final class SwerveModuleConfigs {
        /**
         * This nested static class contains the actual SparkMaxConfig objects for a single MAXSwerve module.
         */
        public static final class MAXSwerveModule {
                /** The configuration object for the driving motor of a swerve module. */
                public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();

                /** The configuration object for the turning (steering) motor of a swerve module. */
                public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

                // A static initializer block is used to configure the SparkMaxConfig objects
                // as soon as this class is loaded.
                static {
                        // Use constants from the Swerve constants file to calculate conversion factors and feedforward gain.
                        double drivingFactor = Swerve.kWheelDiameterMeters * Math.PI
                                        / Swerve.kDrivingMotorReduction;
                        double turningFactor = 2 * Math.PI; // Radians
                        double drivingVelocityFeedForward = 1 / Swerve.kDriveWheelFreeSpeedRps;

                        // Configure the driving motor (the one that spins the wheel).
                        drivingConfig.idleMode(IdleMode.kBrake)
                                        .smartCurrentLimit(Swerve.kDrivingMotorCurrentLimit);
                        drivingConfig.encoder
                                        // Set the conversion factor to convert from motor rotations to linear meters.
                                        .positionConversionFactor(drivingFactor)
                                        // Set the conversion factor to convert from RPM to meters per second.
                                        .velocityConversionFactor(drivingFactor / 60.0);
                        drivingConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                        // Set the PID gains for the driving motor's velocity control.
                                        // Note: These are example gains and may need tuning for your specific robot.
                                        .pid(Swerve.kDrivingP, Swerve.kDrivingI, Swerve.kDrivingD)
                                        // Set the feedforward gain for the driving motor.
                                        .velocityFF(drivingVelocityFeedForward)
                                        // Set the output range for the driving motor.
                                        .outputRange(Swerve.kDrivingMinOutput,
                                                        Swerve.kDrivingMaxOutput);

                        // Configure the turning motor (the one that steers the module).
                        turningConfig.idleMode(IdleMode.kBrake)
                                        .smartCurrentLimit(Swerve.kTurningMotorCurrentLimit);
                        turningConfig.absoluteEncoder
                                        // Invert the turning encoder, as the output shaft rotates in the opposite
                                        // direction of the steering motor in the MAXSwerve Module.
                                        .inverted(Swerve.kTurningEncoderInverted)
                                        // Set the conversion factor to convert from rotations to radians.
                                        .positionConversionFactor(turningFactor)
                                        // Set the conversion factor to convert from RPM to radians per second.
                                        .velocityConversionFactor(turningFactor / 60.0);
                        turningConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                                        // Set the PID gains for the turning motor's position control.
                                        // Note: These are example gains and may need tuning for your specific robot.
                                        .pid(Swerve.kTurningP, Swerve.kTurningI, Swerve.kTurningD)
                                        // Set the output range for the turning motor.
                                        .outputRange(Swerve.kTurningMinOutput,
                                                        Swerve.kTurningMaxOutput)
                                        // Enable PID wrap-around for the turning motor. This allows the PID
                                        // controller to take the shortest path to the setpoint. For example,
                                        // going from 350 degrees to 10 degrees will go through 0 degrees
                                        // rather than taking the long way around.
                                        .positionWrappingEnabled(true)
                                        .positionWrappingInputRange(0, turningFactor);
                }
        }
}
