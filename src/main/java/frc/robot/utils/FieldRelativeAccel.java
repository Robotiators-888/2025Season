package frc.robot.utils;

/**
 * The FieldRelativeAccel class represents the field-relative acceleration of the robot.
 * It is a data structure that stores the linear acceleration in the x and y directions,
 * and the angular acceleration, all relative to the field's coordinate system.
 */
public class FieldRelativeAccel {
    /** The linear acceleration in the x-direction (m/s^2). */
    public double ax;

    /** The linear acceleration in the y-direction (m/s^2). */
    public double ay;

    /** The angular acceleration (rad/s^2). */
    public double alpha;

    /**
     * Constructs a new FieldRelativeAccel with specified acceleration values.
     * @param ax The linear acceleration in the x-direction (m/s^2).
     * @param ay The linear acceleration in the y-direction (m/s^2).
     * @param alpha The angular acceleration (rad/s^2).
     */
    public FieldRelativeAccel(double ax, double ay, double alpha) {
        this.ax = ax;
        this.ay = ay;
        this.alpha = alpha;
    }

    /**
     * Constructs a new FieldRelativeAccel by calculating the acceleration from two
     * sequential FieldRelativeSpeed measurements and the time difference between them.
     * @param newSpeed The newer speed measurement.
     * @param oldSpeed The older speed measurement.
     * @param time The time difference between the two speed measurements in seconds.
     */
    public FieldRelativeAccel(FieldRelativeSpeed newSpeed, FieldRelativeSpeed oldSpeed,
            double time) {
        // Calculate acceleration as the change in velocity over time.
        this.ax = (newSpeed.vx - oldSpeed.vx) / time;
        this.ay = (newSpeed.vy - oldSpeed.vy) / time;
        this.alpha = (newSpeed.omega - oldSpeed.omega) / time;

        // Cap the calculated acceleration values to a reasonable maximum to avoid unrealistic spikes.
        if (Math.abs(this.ax) > 6.0) {
            this.ax = 6.0 * Math.signum(this.ax);
        }
        if (Math.abs(this.ay) > 6.0) {
            this.ay = 6.0 * Math.signum(this.ay);
        }
        if (Math.abs(this.alpha) > 4 * Math.PI) {
            this.alpha = 4 * Math.PI * Math.signum(this.alpha);
        }
    }

    /**
     * Constructs a new FieldRelativeAccel with all acceleration values initialized to zero.
     */
    public FieldRelativeAccel() {
        this.ax = 0.0;
        this.ay = 0.0;
        this.alpha = 0.0;
    }

}
