package frc.robot.subsystems;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import frc.robot.Constants.Swerve;

public final class ShooterConfigs {
        public static final class ShooterIndexIntakeConfigs {
                public static final SparkMaxConfig shootingConfig = new SparkMaxConfig();

                static {
                        // Use module constants to calculate conversion factors and feed forward
                        // gain.
                        double intakeFactor = Swerve.kWheelDiameterMeters * Math.PI
                                        / Swerve.kDrivingMotorReduction; //speed for intake
                        double PivotFactor = 2 * Math.PI; //motion for pivot
                        double ShooterVelocity = 1 / Swerve.kDriveWheelFreeSpeedRps;

                        shootingConfig.idleMode(IdleMode.kBrake)
                                        .smartCurrentLimit(Swerve.kDrivingMotorCurrentLimit);
                        shootingConfig.encoder.positionConversionFactor(intakeFactor) // meters
                                        .velocityConversionFactor(intakeFactor / 60.0); // meters
                                                                                         // per
                                                                                         // second
                        shootingConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                        // These are example gains you may need to them for your own
                                        // robot!
                                        .pid(Swerve.kDrivingP, Swerve.kDrivingI, Swerve.kDrivingD)
                                        .velocityFF(ShooterVelocity)
                                        .outputRange(Swerve.kDrivingMinOutput,
                                                        Swerve.kDrivingMaxOutput);
                }
        }
}
