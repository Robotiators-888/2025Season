package frc.robot.utils;

import frc.robot.subsystems.SUB_Drivetrain;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;


/**
 * The AutoGenerator class is responsible for configuring the PathPlanner AutoBuilder,
 * which is the core of the autonomous path-following system. It sets up the necessary
 * suppliers and controllers that PathPlanner needs to interface with the robot's drivetrain.
 * This class follows a singleton pattern.
 */
public class AutoGenerator extends SubsystemBase {
    /** The singleton instance of the drivetrain subsystem. */
    public static SUB_Drivetrain drivetrain = SUB_Drivetrain.getInstance();

    /** The singleton instance of the AutoGenerator. */
    private static AutoGenerator INSTANCE = null;

    /** A flag indicating whether the robot has reached a specific target during an autonomous path. */
    public static boolean reachedAutoTarget;

    /** A flag indicating whether a game piece intake process is complete during an autonomous path. */
    public static boolean intakeComplete = true;

    /**
     * Constructs a new AutoGenerator.
     * This constructor is private to enforce the singleton pattern.
     * It configures the PathPlanner AutoBuilder with suppliers for robot pose, chassis speeds,
     * and methods for driving and resetting odometry. It also sets the PID constants for path following.
     */
    public AutoGenerator() {
        RobotConfig config;
        try {
            // Attempt to load the robot configuration from the PathPlanner GUI settings.
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
            // If loading fails, return to prevent further errors.
            return;
        }
        // Configure the AutoBuilder with the necessary suppliers and controllers.
        AutoBuilder.configure(
                drivetrain::getPose, // A supplier for the robot's current pose on the field.
                drivetrain::resetPose, // A method to reset the robot's odometry to a given pose.
                drivetrain::getChassisSpeeds, // A supplier for the robot's current robot-relative chassis speeds.
                drivetrain::driveRobotRelative, // A method that drives the robot given robot-relative chassis speeds.
                new PPHolonomicDriveController( // The built-in path-following controller for holonomic drivetrains.
                        new PIDConstants(10, 0.0, 0.0), // PID constants for translation control.
                        new PIDConstants(12, 6.0, 0.0) // PID constants for rotation control.
                ),
                config, // The robot configuration loaded from the GUI.
                () -> {
                    // A boolean supplier that returns true if the current alliance is Red.
                    // This is used to automatically flip paths based on alliance color.
                    return DriverStation.getAlliance().equals(Optional.of(Alliance.Red));
                },
                drivetrain // The drivetrain subsystem, to set requirements for path-following commands.
        );

        registerAllCommands();
    }

    /**
     * Registers all named commands that can be used in PathPlanner autonomous paths.
     * These commands allow for actions like intaking, scoring, or running other subsystems
     * to be embedded directly into the path files created in the PathPlanner GUI.
     * Example: NamedCommands.registerCommand("shoot", new CMD_Shoot());
     */
    public void registerAllCommands() {
        // Register custom commands here.
    }

    /**
     * Returns the singleton instance of the AutoGenerator.
     * @return The singleton instance of AutoGenerator.
     */
    public static AutoGenerator getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new AutoGenerator();
        }

        return INSTANCE;
    }

    /**
     * Sets the flag indicating that the robot has reached an autonomous target.
     * @param value The new boolean value for the flag.
     */
    public void setReachedTarget(boolean value) {
        reachedAutoTarget = value;
        SmartDashboard.putBoolean("ReachedAutoTarget", reachedAutoTarget);
    }

    /**
     * Returns the current value of the reachedAutoTarget flag.
     * @return True if the robot has reached the target, false otherwise.
     */
    public boolean getReachedTarget() {
        return reachedAutoTarget;
    }

    /**
     * Sets the flag indicating that the intake process is complete.
     * @param value The new boolean value for the flag.
     */
    public void setIntakeComplete(boolean value) {
        intakeComplete = value;
        SmartDashboard.putBoolean("IntakeComplete", intakeComplete);
    }

    /**
     * Returns the current value of the intakeComplete flag.
     * @return True if the intake process is complete, false otherwise.
     */
    public boolean getIntakeComplete() {
        return intakeComplete;
    }

    /**
     * This method is called periodically every robot loop. It is currently empty.
     */
    @Override
    public void periodic() {
    }
}
