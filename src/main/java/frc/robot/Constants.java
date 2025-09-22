// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * as `public static final` so they are immutable and accessible globally.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

        /**
         * The Operator class contains constants related to the operator interface, such as controller ports and deadbands.
         */
        public static class Operator {
                /** The USB port for the primary driver's controller. */
                public static final int kDriver1ControllerPort = 0;

                /** The USB port for the secondary driver's (operator's) controller. */
                public static final int kDriver2ControllerPort = 1;

                /** The deadband for the drive joysticks to prevent unintentional movement. */
                public static final double kDriveDeadband = 0.05;
        }

        /**
         * The Swerve class contains constants for the swerve module configuration, including PID gains and motor settings.
         */
        public static class Swerve {
                /** The number of teeth on the driving motor's pinion gear. */
                public static final int kDrivingMotorPinionTeeth = 12;

                /** Whether the turning encoder's reading is inverted. */
                public static final boolean kTurningEncoderInverted = true;

                /** The free (no-load) speed of the driving motor in rotations per second. */
                public static final double kDrivingMotorFreeSpeedRps =
                                Motor.kVortexFreeSpeedRpm / 60;

                /** The diameter of the swerve module wheels in meters. */
                public static final double kWheelDiameterMeters = Units.inchesToMeters(2 * 1.6243455433105947);

                /** The circumference of the swerve module wheels in meters. */
                public static final double kWheelCircumferenceMeters =
                                kWheelDiameterMeters * Math.PI;

                /** The overall gear reduction for the driving motor. */
                public static final double kDrivingMotorReduction =
                                (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);

                /** The theoretical maximum speed of the drive wheel in rotations per second. */
                public static final double kDriveWheelFreeSpeedRps =
                                (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
                                                / kDrivingMotorReduction;

                /** The conversion factor from driving encoder rotations to linear meters. */
                public static final double kDrivingEncoderPositionFactor =
                                (kWheelDiameterMeters * Math.PI) / kDrivingMotorReduction; // meters

                /** The conversion factor from driving encoder RPM to meters per second. */
                public static final double kDrivingEncoderVelocityFactor =
                                ((kWheelDiameterMeters * Math.PI) / kDrivingMotorReduction) / 60.0; // meters per second

                /** The conversion factor from turning encoder rotations to radians. */
                public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians

                /** The conversion factor from turning encoder RPM to radians per second. */
                public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

                /** The minimum input for the turning encoder position PID controller (radians). */
                public static final double kTurningEncoderPositionPIDMinInput = 0;

                /** The maximum input for the turning encoder position PID controller (radians). */
                public static final double kTurningEncoderPositionPIDMaxInput =
                                kTurningEncoderPositionFactor;

                /** The Proportional gain (P) for the driving PID controller. */
                public static final double kDrivingP = 0.04;

                /** The Integral gain (I) for the driving PID controller. */
                public static final double kDrivingI = 0;

                /** The Derivative gain (D) for the driving PID controller. */
                public static final double kDrivingD = 0;

                /** The Feedforward gain (FF) for the driving PID controller. */
                public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;

                /** The minimum output for the driving PID controller. */
                public static final double kDrivingMinOutput = -1;

                /** The maximum output for the driving PID controller. */
                public static final double kDrivingMaxOutput = 1;

                /** The Proportional gain (P) for the turning PID controller. */
                public static final double kTurningP = 1;

                /** The Integral gain (I) for the turning PID controller. */
                public static final double kTurningI = 0;

                /** The Derivative gain (D) for the turning PID controller. */
                public static final double kTurningD = 0;

                /** The Feedforward gain (FF) for the turning PID controller. */
                public static final double kTurningFF = 0;

                /** The minimum output for the turning PID controller. */
                public static final double kTurningMinOutput = -1;

                /** The maximum output for the turning PID controller. */
                public static final double kTurningMaxOutput = 1;

                /** The tolerance for the heading PID controller in radians. */
                public static final double headingTolerance = Degrees.of(1).in(Radians);

                /** The maximum rotational speed of the robot in radians per second. */
                public static final double kMaxRotationalSpeed =
                                (kDrivingMotorFreeSpeedRps / 60) / Drivetrain.kTrackRadius;

                /** The idle mode for the driving motor (Brake or Coast). */
                public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;

                /** The idle mode for the turning motor (Brake or Coast). */
                public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

                /** The smart current limit for the driving motor in amps. */
                public static final int kDrivingMotorCurrentLimit = 60; // amps

                /** The smart current limit for the turning motor in amps. */
                public static final int kTurningMotorCurrentLimit = 20; // amps

        }

        /**
         * The Drivetrain class contains constants for the drivetrain subsystem, such as motor CAN IDs and physical dimensions.
         */
        public static final class Drivetrain {
                // --- CAN IDs --- //
                public static final int kFRONT_LEFT_DRIVE_MOTOR_CANID = 20;
                public static final int kFRONT_LEFT_STEER_MOTOR_CANID = 21;
                public static final int kFRONT_RIGHT_DRIVE_MOTOR_CANID = 22;
                public static final int kFRONT_RIGHT_STEER_MOTOR_CANID = 23;
                public static final int kBACK_RIGHT_DRIVE_MOTOR_CANID = 24;
                public static final int kBACK_RIGHT_STEER_MOTOR_CANID = 25;
                public static final int kBACK_LEFT_DRIVE_MOTOR_CANID = 26;
                public static final int kBACK_LEFT_STEER_MOTOR_CANID = 27;

                // --- Orientations --- //
                public static final Rotation2d shooterSide = new Rotation2d(0); // 0 degrees
                public static final Rotation2d intakeSide = new Rotation2d(Math.PI); // 180 degrees

                // --- Kinematic Limits --- //
                public static final double kMaxSpeedMetersPerSecond = 5.74;
                public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

                // --- Slew Rate Limiters for Smoother Driving --- //
                public static final double kDirectionSlewRate = 100000; // radians per second
                public static final double kMagnitudeSlewRate = 100000; // percent per second (1 = 100%)
                public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

                // --- Physical Dimensions --- //
                public static final double kTrackWidth = Units.inchesToMeters(23.5); // Distance between centers of right and left wheels.
                public static final double kWheelBase = Units.inchesToMeters(27); // Distance between centers of front and back wheels.
                public static final double kTrackRadius = Units.inchesToMeters(17.8972763); // Distance from center of robot to a wheel.

                /** The kinematics object for the swerve drive, defining the module positions relative to the robot center. */
                public static final SwerveDriveKinematics kDriveKinematics =
                                new SwerveDriveKinematics(
                                                new Translation2d(kWheelBase / 2, kTrackWidth / 2),   // Front Left
                                                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),  // Front Right
                                                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),   // Back Left
                                                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)); // Back Right

                // --- Module Angular Offsets --- //
                public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2.0;
                public static final double kFrontRightChassisAngularOffset = 0.0;
                public static final double kBackLeftChassisAngularOffset = Math.PI;
                public static final double kBackRightChassisAngularOffset = Math.PI / 2.0;

                // --- Gyro Configuration --- //
                public static final boolean kGyroReversed = true;
                public static final double kGyroRotation = 0; // Unused

                // --- Alignment Constants --- //
                public static final double kXShiftMagnitude =
                                Units.inchesToMeters(5+(30.5 / 2));
                public static final double kYShiftMagnitude = Units.inchesToMeters(6.5);

        }

        /**
         * The Motor class contains constants for different types of motors used on the robot.
         */
        public static final class Motor {
                /** The free (no-load) speed of a REV Vortex motor in RPM. */
                public static final double kVortexFreeSpeedRpm = 6784;

                /** The free (no-load) speed of a REV NEO motor in RPM. */
                public static final double kNeoFreeSpeedRpm = 5676;
        }

        /**
         * The Field class contains constants for the game field's dimensions.
         */
        public static final class Field {
                /** The length of the field in meters. */
                public static final double fieldLength = 17.55; // 1755.0 / 100.0

                /** The width of the field in meters. */
                public static final double fieldWidth = 8.05; // 805.0 / 100.0
        }

        /**
         * The PhotonVision class contains constants for the PhotonVision cameras, including names and transformations.
         */
        public static final class PhotonVision {

                // --- Camera 1 --- //
                public static final String kCam1Name = "AprilTagCam1";
                public static final Rotation3d cameraRotation = new Rotation3d(
                                Units.degreesToRadians(0), Units.degreesToRadians(0),
                                Units.degreesToRadians(-25));
                public static final Transform3d kRobotToCamera1 = new Transform3d(
                                Units.inchesToMeters(15.25 - 7.625), Units.inchesToMeters(13.5 - 2.75),
                                Units.inchesToMeters(11), cameraRotation);

                // --- Camera 2 --- //
                public static final String kCam2Name = "AprilTagCam2";
                public static final Rotation3d cameraRotation2 = new Rotation3d(
                                Units.degreesToRadians(0), Units.degreesToRadians(0),
                                Units.degreesToRadians(25));
                public static final Transform3d kRobotToCamera2 = new Transform3d(
                                Units.inchesToMeters(15.25-7.625), Units.inchesToMeters(-13.5+2.75),
                                Units.inchesToMeters(11), cameraRotation2);

                // --- Camera 3 --- //
                public static final String kCam3Name = "AprilTagHighCam";
                public static final Rotation3d cameraRotation3 = new Rotation3d(0,
                                 Units.degreesToRadians(0), Units.degreesToRadians(8));
                public static final Transform3d kRobotToCamera3 = new Transform3d(
                                 Units.inchesToMeters(-7+3.25), Units.inchesToMeters(-10),
                                 Units.inchesToMeters(23.5), cameraRotation);
        }

        /**
         * The PivotConstants class contains constants for the pivot subsystem, including setpoints and feedforward values.
         */
        public static final class PivotConstants {
                /** The feedforward gains for the arm when it is not carrying coral. */
                public static final ArmFeedforward noCoralArmFeedforward =
                                new ArmFeedforward(0, 0.69, 0.34);

                /** The feedforward gains for the arm when it is carrying coral. */
                public static final ArmFeedforward coralArmFeedforward =
                                new ArmFeedforward(0, 0.69, 0.34);

                // --- Setpoints (in degrees) --- //
                public static final double kAlgaeSafeSetpoint = 160;
                public static final double kAlgaeScoringSetpoint = 132;
                public static final double kIntakeSetpoint = 326;
                public static final double kElevatingSetpoint = 288;
                public static final double kCoralSetpoint = 300;
                public static final double kAlgaeSetpoint = 176;
                public static final double kL1Setpoint = 300;
                public static final double kL2Setpoint = 294;
                public static final double kL3Setpoint = 295;
                public static final double kL4Setpoint = 273;

                /** The tolerance for the pivot's position in degrees. */
                public static final double toleranceDegrees = 5;

                /** The upper bound of a known 'stuck' point for the pivot mechanism. */
                public static final double kUpperBoundStuckPoint = 340.0;

                /** The lower bound of a known 'stuck' point for the pivot mechanism. */
                public static final double kLowerBoundStuckPoint = 327.0;
        }

        /**
         * The Elevator class contains constants for the elevator subsystem, including voltages, setpoints, and tolerances.
         */
        public static final class Elevator {
                // --- Homing Constants --- //
                public static final double kHomingEncoderLocation = 0; // The encoder value after homing.
                public static final double kResetHomingThreshold = 0.05; // Position threshold to reset homing status.
                public static final double kEncoderNearZero = 0.01; // Position threshold to be considered 'near zero'.
                public static final double kHomingEmergencyCurrent = 40; // Amps; current threshold for emergency homing.
                public static final double kHomingVoltage = -0.25; // Volts; voltage for homing sequence.

                /** The tolerance for the elevator's position in meters. */
                public static final double kTolerance = 0.05;

                // --- Upward Movement Voltages --- //
                public static final double kMaxUpVoltage = 6.375;
                public static final double kMaxUpErrorThreshold = 0.25; // meters
                public static final double kHighUpVoltage = 5.1;
                public static final double kHighUpErrorThreshold = 0.15; // meters
                public static final double kMediumUpVoltage = 4.2;
                public static final double kMediumUpErrorThreshold = 0.06; // meters
                public static final double kSlowUpVoltage = 3.4;

                // --- Downward Movement Voltages --- //
                public static final double kMaxDownVoltage = -2.65;
                public static final double kMaxDownErrorThreshold = 0.25; // meters
                public static final double kHighDownVoltage = -2.1;
                public static final double kHighDownErrorThreshold = 0.20; // meters
                public static final double kMediumDownVoltage = -1.2;
                public static final double kMediumDownErrorThreshold = 0.09; // meters
                public static final double kSlowDownVoltage = -0.55;
                public static final double kSlowDownThreshold = 0.06; // meters

                // --- Holding Voltages --- //
                public static final double kEmptyHoldingVoltage = 0.6;
                public static final double kEmptyHoldingVoltageTop = 0.74;
                public static final double kCoralHoldingVoltage = 0.85;
                public static final double kAlgaeHoldingVoltage = 0.72;

                // --- Setpoints (in meters) --- //
                public static final double kStartingSetpoint = 0;
                public static final double kL1Setpoint = 0.1;
                public static final double kL2Setpoint = 0.162;
                public static final double kL3Setpoint = 0.375;
                public static final double kL4Setpoint = 0.705;
                public static final double kAlgaeSetpoint = 0.508;
                public static final double kProcessorSetpoint = 0.104;
        }

        /**
         * The Roller class contains constants for the roller subsystem, used for intake and outtake.
         */
        public static class Roller {
                public static final int kRollerCanID = 30;
                public static final int kHelperCanID = 34;

                // --- Control Constants --- //
                public static final double kIntakeCurrentThreshold = 35; // Amps
                public static final int kRollerCurrentLimit = 60; // Amps
                public static final int kHelperCurrentLimit = 20; // Amps
                public static final double kRollerHelperSpeed = 0.7; // Percent
                public static final double kIntakeSpeed = 0.2; // Percent
                public static final double kIntakeFinishSpeed = 0.1; // Percent
                public static final double kIntakeStartingTime = 1.25; // Seconds
                public static final double kIntakeFinishTime = 0.12; // Seconds
                public static final double kEjectSpeed = 0.7; // Percent
                public static final double kFreeSpinThreshold = 420; // RPM
        }

        /**
         * The Climber class contains constants for the climber subsystem.
         */
        public static class Climber {
                public static final int kClimberCanID = 40;
                public static final double kClimberPercentOutput = 0.2;
        }

        /**
         * The LEDs class contains constants for the LED subsystem.
         */
        public static class LEDs {
                /** The PWM port the Blinkin LED controller is connected to. */
                public static final int kPWMPort = 9;

                /** The PWM value for the color green. */
                public static final double kColorGreen = 0.77;

                /** The PWM value for the color red. */
                public static final double kColorRed = 0.61;

                /** The PWM value for the party palette twinkles pattern. */
                public static final double kParty_Palette_Twinkles = -0.53;
        }
}
