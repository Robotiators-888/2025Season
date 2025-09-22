package frc.robot.utils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * The FieldRelativeSpeed class represents the field-relative speed of the robot.
 * It is a data structure that stores the linear speed in the x and y directions,
 * and the angular speed, all relative to the field's coordinate system.
 */
public class FieldRelativeSpeed {
    /** The linear speed in the x-direction (m/s). */
    public double vx;

    /** The linear speed in the y-direction (m/s). */
    public double vy;

    /** The angular speed (rad/s). */
    public double omega;

    /**
     * Constructs a new FieldRelativeSpeed with specified speed values.
     * @param vx The linear speed in the x-direction (m/s).
     * @param vy The linear speed in the y-direction (m/s).
     * @param omega The angular speed (rad/s).
     */
    public FieldRelativeSpeed(double vx, double vy, double omega) {
        this.vx = vx;
        this.vy = vy;
        this.omega = omega;
    }

    /**
     * Constructs a new FieldRelativeSpeed from a robot-relative ChassisSpeeds object and a gyro angle.
     * This converts the robot's local speeds into speeds relative to the field.
     * @param chassisSpeed The robot's chassis speeds (robot-relative).
     * @param gyro The robot's gyro angle (heading).
     */
    public FieldRelativeSpeed(ChassisSpeeds chassisSpeed, Rotation2d gyro) {
        // This is the standard rotation matrix transformation from robot-relative to field-relative coordinates.
        this(chassisSpeed.vxMetersPerSecond * gyro.getCos()
                - chassisSpeed.vyMetersPerSecond * gyro.getSin(),
                chassisSpeed.vyMetersPerSecond * gyro.getCos()
                        + chassisSpeed.vxMetersPerSecond * gyro.getSin(),
                chassisSpeed.omegaRadiansPerSecond);
    }

    /**
     * Constructs a new FieldRelativeSpeed with all speed values initialized to zero.
     */
    public FieldRelativeSpeed() {
        this.vx = 0.0;
        this.vy = 0.0;
        this.omega = 0.0;
    }

}
