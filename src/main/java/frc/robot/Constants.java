// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

        public static class OperatorConstants {
                public static final int kDriver1ControllerPort = 0;
                public static final int kDriver2ControllerPort = 1;
                public static final double kDriveDeadband = 0.2;
        }

        public static class Swerve {
                // The MAXSwerve module can be configured with one of three pinion gears: 12T,
                // 13T, or 14T.
                // This changes the drive speed of the module (a pinion gear with more teeth
                // will result in a
                // robot that drives faster).
                public static final int kDrivingMotorPinionTeeth = 14;

                // Invert the turning encoder, since the output shaft rotates in the opposite
                // direction of
                // the steering motor in the MAXSwerve Module.
                public static final boolean kTurningEncoderInverted = true;

                // Calculations required for driving motor conversion factors and feed forward
                public static final double kDrivingMotorFreeSpeedRps =
                                MotorConstants.kVortexFreeSpeedRpm / 60;
                public static final double kWheelDiameterMeters = Units.inchesToMeters(2.90);
                // Thrifty tread 2.95in
                // Orange Tread 2.70
                // Black Rev 2.95
                public static final double kWheelCircumferenceMeters =
                                kWheelDiameterMeters * Math.PI;
                // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
                // teeth on the
                // bevel pinion
                public static final double kDrivingMotorReduction =
                                (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
                public static final double kDriveWheelFreeSpeedRps =
                                (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
                                                / kDrivingMotorReduction;

                public static final double kDrivingEncoderPositionFactor =
                                (kWheelDiameterMeters * Math.PI) / kDrivingMotorReduction; // meters
                public static final double kDrivingEncoderVelocityFactor =
                                ((kWheelDiameterMeters * Math.PI) / kDrivingMotorReduction) / 60.0; // meters
                                                                                                    // per
                                                                                                    // second

                public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians

                public static final double kTurningVelocityFactor = (2 * Math.PI) / 60.0; // radians
                                                                                          // per
                                                                                          // second
                public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians
                                                                                                 // per
                                                                                                 // second

                public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
                public static final double kTurningEncoderPositionPIDMaxInput =
                                kTurningEncoderPositionFactor; // radians

                public static final double kDrivingP = 0.04;
                public static final double kDrivingI = 0;
                public static final double kDrivingD = 0;
                public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
                public static final double kDrivingMinOutput = -1;
                public static final double kDrivingMaxOutput = 1;

                public static final double kTurningP = 1;
                public static final double kTurningI = 0;
                public static final double kTurningD = 0;
                public static final double kTurningFF = 0;
                public static final double kTurningMinOutput = -1;
                public static final double kTurningMaxOutput = 1;
                public static final double headingTolerance = Degrees.of(1).in(Radians);
                // Max Rot = Max Linear ((meters/sec)/60 (m/s)) / radius
                public static final double kMaxRotationalSpeed =
                                (kDrivingMotorFreeSpeedRps / 60) / Drivetrain.kTrackRadius;

                public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
                public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

                public static final int kDrivingMotorCurrentLimit = 60; // amps
                public static final int kTurningMotorCurrentLimit = 20; // amps

        }

        public static final class Drivetrain {
                public static final int kFRONT_LEFT_DRIVE_MOTOR_CANID = 20;
                public static final int kFRONT_LEFT_STEER_MOTOR_CANID = 21;
                public static final int kFRONT_RIGHT_DRIVE_MOTOR_CANID = 22;
                public static final int kFRONT_RIGHT_STEER_MOTOR_CANID = 23;
                public static final int kBACK_RIGHT_DRIVE_MOTOR_CANID = 24;
                public static final int kBACK_RIGHT_STEER_MOTOR_CANID = 25;
                public static final int kBACK_LEFT_DRIVE_MOTOR_CANID = 26;
                public static final int kBACK_LEFT_STEER_MOTOR_CANID = 27;

                public static final Rotation2d shooterSide = new Rotation2d(0);
                public static final Rotation2d intakeSide = new Rotation2d(180);

                // Driving Parameters - Note that these are not the maximum capable speeds of
                // the robot, rather the allowed maximum speeds
                public static final double kMaxSpeedMetersPerSecond = 5.74;
                public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

                public static final double kDirectionSlewRate = 100000; // radians per second
                public static final double kMagnitudeSlewRate = 100000; // percent per second (1 =
                                                                        // 100%)
                public static final double kRotationalSlewRate = 2.0; // percent per second (1 =
                                                                      // 100%)

                // Chassis configuration
                public static final double kTrackWidth = Units.inchesToMeters(24);
                // 31inches by 24inches
                // Distance between centers of right and left wheels on robot
                public static final double kWheelBase = Units.inchesToMeters(31);

                public static final double kTrackRadius =
                                Units.inchesToMeters(19.6 * Math.sqrt(2) / 2);
                public static final double kMaxModuleSpeed = Units.feetToMeters(15);
                // Distance between front and back wheels on robot
                public static final SwerveDriveKinematics kDriveKinematics =
                                new SwerveDriveKinematics(
                                                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                                                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                                                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                                                new Translation2d(-kWheelBase / 2,
                                                                -kTrackWidth / 2));

                // Angular offsets of the modules relative to the chassis in radians
                public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2.0;
                public static final double kFrontRightChassisAngularOffset = 0.0;
                public static final double kBackLeftChassisAngularOffset = Math.PI;
                public static final double kBackRightChassisAngularOffset = Math.PI / 2.0;

                public static final boolean kGyroReversed = true;

                public static final double kGyroRotation = 0;
        }

        // Motor Constants
        public static final class MotorConstants {
                public static final double kVortexFreeSpeedRpm = 6784;
                public static final double kNeoFreeSpeedRpm = 5676;
        }

        public static final class FieldConstants {
                public static final double fieldLength = Units.inchesToMeters(648);
                public static final double fieldWidth = Units.inchesToMeters(324);

                public static final Translation2d speakerAimPoint =
                                new Translation2d(0.240581, 5.547755);

                public static final Pose2d subwooferFront = new Pose2d(
                                new Translation2d(1.45, 5.55), Rotation2d.fromDegrees(+180));
                public static final Pose2d subwooferAmp = new Pose2d(new Translation2d(0.71, 6.72),
                                Rotation2d.fromDegrees(-120));
                public static final Pose2d subwooferSource = new Pose2d(
                                new Translation2d(0.71, 4.57), Rotation2d.fromDegrees(+120));

                public static final Pose2d amp = new Pose2d(new Translation2d(1.83, 7.61),
                                Rotation2d.fromDegrees(-90));
                public static final Pose2d podium = new Pose2d(new Translation2d(2.76, 4.44),
                                Rotation2d.fromDegrees(+157.47));

                public static final Pose2d pathfindSpeaker = new Pose2d(
                                new Translation2d(3.45, 5.55), Rotation2d.fromDegrees(+180));
                public static final Pose2d pathfindSource = new Pose2d(
                                new Translation2d(13.41, 1.54), Rotation2d.fromDegrees(+180));

                public static final double podiumToSpeakerDist =
                                speakerAimPoint.getDistance(podium.getTranslation());
                public static final double subwooferToSpeakerDist =
                                speakerAimPoint.getDistance(subwooferFront.getTranslation());
        }


        public static final class PivotConstants {
                public static final ArmFeedforward noCoralArmFeedforward =
                                new ArmFeedforward(0, 0.69, 0.34);
                public static final ArmFeedforward coralArmFeedforward =
                                new ArmFeedforward(0, 0.69, 0.34); // TODO: Set it the same for now,
                                                                   // change later

                public static final double kIntakeSetpoint = 324.88;
                public static final double kElevatingSetpoint = 273.5;
                public static final double kCoralSetpoint = 300;
                public static final double kAlgaeSetpoint = 156;
                public static final double toleranceDegrees = 0.0;
        }

        public static final class Elevator {
                public static final double kHomingEncoderLocation = 0;
                public static final double kResetHomingThreshold = 1000;
                public static final double kEncoderNearZero = 10;

                public static final double kHomingEmergencyCurrent = 20;
                public static final boolean kStartingHoming = false;
                public static final double kHomingSpeed = 0.3;
                public static final double kTolerance = .05;
                public static final double kPIDTolerance = 20;
                public static final double kTimeStep = 0.02;

                public static final double kStartingSetpoint = 0;
                public static final double kL1Setpoint = .15;
                public static final double kL2Setpoint = .35;
                public static final double kL3Setpoint = .5;
                public static final double kL4Setpoint = .805;


                public static final double kP = 0;
                public static final double kI = 0;
                public static final double kD = 0;

                public static final double kG = 0.6;
                public static final double kS = 0.3;
                public static final double kV = 5.00;
        }

        public static class Roller {
                public static final int kRollerCanID = 30;
                public static final double kIntakeCurrentThreshold = 35; // Amps
                public static final int kRollerCurrentLimit = 60;

                public static final double kIntakeSpeed = 0.25; // Percent
                public static final double kIntakeFinishSpeed = 0.1; // Percent
                public static final double kIntakeStartingTime = 1.25; // Seconds
                public static final double kIntakeFinishTime = 0.1; // Seconds

                public static final double kEjectSpeed = 0.8; // Percent
                public static final double kFreeSpinThreshold = 6000; // RPM
        }
        public static class Climber {
                public static final int kClimberCanID = 40;
                public static final double kClimberPercentOutput = 0.2;
        }
}
