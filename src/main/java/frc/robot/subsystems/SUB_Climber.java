package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Climber;


/**
 * The SUB_Climber class manages the robot's climber mechanism, which is used
 * to elevate the robot at the end of the match. It controls a single motor
 * responsible for the climbing action and uses a limit switch to detect the
 * lower bound of the mechanism. This class follows a singleton pattern to
 * ensure only one instance is created throughout the robot's lifecycle.
 */
public class SUB_Climber extends SubsystemBase {
    /** The singleton instance of the climber subsystem. */
    private static SUB_Climber INSTANCE = null;

    /** The SparkMax motor controller for the climber mechanism. */
    private SparkMax climber = new SparkMax(Climber.kClimberCanID, MotorType.kBrushless);

    /** The lower limit switch, which is a forward limit switch on the SparkMax. */
    private SparkLimitSwitch lowerLimitSwitch = climber.getForwardLimitSwitch();

    /** The configuration object for the SparkMax motor controller. */
    private SparkMaxConfig climberConfig = new SparkMaxConfig();

    /**
     * Constructs a new SUB_Climber.
     * This constructor is private to enforce the singleton pattern.
     * It configures the climber motor with inversion, voltage compensation, and a smart current limit.
     */
    public SUB_Climber() {
        climberConfig.inverted(false);
        climberConfig.voltageCompensation(12);
        climberConfig.smartCurrentLimit(60);
        climber.configure(climberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Sets the speed of the climber motor.
     *
     * @param speed The desired speed of the motor, ranging from -1.0 (downward) to 1.0 (upward).
     */
    public void setSpeed(double speed) {
        climber.set(speed);
    }

    /**
     * Returns the singleton instance of the climber subsystem.
     * This method ensures that only one instance of the SUB_Climber is created and used.
     *
     * @return The singleton instance of the SUB_Climber.
     */
    public static SUB_Climber getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new SUB_Climber();
        }
        return INSTANCE;
    }

    /**
     * This method is called periodically every robot loop (approximately every 20ms).
     * It can be used for tasks that need to be run continuously, such as updating
     * sensor readings or dashboard values. Currently, it is empty.
     */
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}