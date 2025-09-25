// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;

import org.json.simple.parser.ParseException;
import org.photonvision.EstimatedRobotPose;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Climber;
import frc.robot.Constants.Elevator;
import frc.robot.Constants.Field;
import frc.robot.Constants.LEDs;
import frc.robot.Constants.Operator;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.Roller;
import frc.robot.commands.CMD_OldPathfindReefAlign;
import frc.robot.commands.CMD_PathfindAlgaeAlign;
import frc.robot.commands.CMD_PathfindReefAlign;
import frc.robot.subsystems.SUB_Climber;
import frc.robot.subsystems.SUB_Drivetrain;
import frc.robot.subsystems.SUB_Elevator;
import frc.robot.subsystems.SUB_LEDs;
import frc.robot.subsystems.SUB_PhotonVision;
import frc.robot.subsystems.SUB_Pivot;
import frc.robot.subsystems.SUB_Roller;
import frc.robot.utils.AutoGenerator;
import frc.robot.utils.Elastic;


/**
 * This class is where the bulk of the robot's functionality is declared and configured.
 * Since Command-based is a "declarative" paradigm, very little robot logic should actually be
 * handled in the {@link Robot} periodic methods (other than the scheduler calls). Instead, the
 * structure of the robot (including subsystems, commands, and trigger mappings) should be
 * declared here.
 */
public class RobotContainer {
        // The robot's subsystems and commands are defined here...

        // Subsystem Instantiation
        // Notice the getInstance() method. This is how we make sure that there is only one instance of each subsystem.
        private static final SUB_Drivetrain drivetrain = SUB_Drivetrain.getInstance();
        private static final SUB_PhotonVision photonVision = SUB_PhotonVision.getInstance();
        private static final AutoGenerator autoGenerator = AutoGenerator.getInstance();
        public static final SUB_Elevator elevator = SUB_Elevator.getInstance();
        public static final SUB_Roller roller = SUB_Roller.getInstance();
        public static final SUB_Pivot pivot = SUB_Pivot.getInstance(roller.getAbsoluteEncoder());
        public static final SUB_Climber climber = SUB_Climber.getInstance();
        public static final SUB_LEDs leds = SUB_LEDs.getInstance();

        /** The Power Distribution Panel, for monitoring current and voltage. */
        public static final PowerDistribution powerDistribution = new PowerDistribution();

        // Autonomous Mode Selection
        private final SendableChooser<Command> autoChooser;
        private static String autoName, newAutoName;

        // Alliance Color Tracking
        private Optional<Alliance> lastAlliance;
        private Optional<Alliance> alliance;

        /** A Field2d object for visualizing the robot's position and autonomous paths on SmartDashboard. */
        public static final Field2d autoField = new Field2d();

        // Operator Interface
        private final CommandXboxController Driver1 =
                        new CommandXboxController(Operator.kDriver1ControllerPort);
        private final CommandXboxController Driver2 =
                        new CommandXboxController(Operator.kDriver2ControllerPort);

        // State tracking for alignment
        private int listIndex = 0;
        private int targetId = 7;

        //@avacado-a Please review this section for accuracy and clarity

        // For new coders: You will se a lot of () -> in the code below. This is called a lambda expression.
        // Labmdas are used to pass functions as parameters to other functions (especially wpilib functions). They are similar to function pointers in C/C++ (dont worry about c++ yet).
        // Lamdas are technically "one time use" functions that have no name and are anonymous as they are declared and used once.
        // Instant commands are commands that do one thing once and then end
        // Runcommands are commands that run a function repeatedly until interrupted or ended
        // Sequential command groups run a list of commands in order, one after the other
        // Parallel command groups run a list of commands at the same time until they are all finished
        // Race command groups run a list of commands at the same time until one of them finishes, then they end all the other commands
        // Default commands are commands that run on a subsystem when no other command is using/requiring the subsystem. This is seen in the drive subsystem where the default behavior is to use the joysticks to drive
        // When you see a lambda in an instant or run command with a comma and then a subsystem, it means that subsysem is being required (the function and subsystem are parameters of run and instant commands).
        // This works because subsystems inherit from subsystembase

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         * This constructor is where subsystems are initialized, default commands are set,
         * named commands for autonomous are registered, and button bindings are configured.
         */
        public RobotContainer() {
                // Set default commands for subsystems. These run when no other command is scheduled for the subsystem.
                drivetrain.setDefaultCommand(new RunCommand(
                                () -> drivetrain.drive(
                                                MathUtil.applyDeadband(Driver1.getRawAxis(1), Operator.kDriveDeadband),
                                                MathUtil.applyDeadband(Driver1.getRawAxis(0), Operator.kDriveDeadband),
                                                -MathUtil.applyDeadband(Driver1.getRawAxis(4), Operator.kDriveDeadband),
                                                true, true),
                                drivetrain));

                elevator.setDefaultCommand(new RunCommand(
                                () -> elevator.runElevator(() -> pivot.atElevatingSetpoint()),
                                elevator));

                pivot.setDefaultCommand(new RunCommand(
                                () -> pivot.runPivot(() -> roller.getHasCoral()), pivot));

                roller.setDefaultCommand(
                                new RunCommand(() -> roller.setRollerOutput(0.0, 0.0), roller));

                // Configure a "turbo" mode for the drivetrain, activated by the right bumper.
                // This squares the inputs for finer control at low speeds.
                Driver1.rightBumper().whileTrue(new RunCommand(
                                () -> drivetrain.drive(MathUtil.applyDeadband(
                                                Math.copySign(Math.pow(Driver1.getRawAxis(1), 2), Driver1.getRawAxis(1)),
                                                Operator.kDriveDeadband),
                                                MathUtil.applyDeadband(Math.copySign(
                                                                Math.pow(Driver1.getRawAxis(0), 2), Driver1.getRawAxis(0)),
                                                                Operator.kDriveDeadband),
                                                -MathUtil.applyDeadband(Driver1.getRawAxis(4), Operator.kDriveDeadband),
                                                false, true),
                                drivetrain));

                // Register named commands for autonomous mode. These can be called by name from PathPlanner autos.
                registerNamedCommands();

                // Configure the trigger bindings for the controllers.
                configureBindings();

                // Build the autonomous chooser and add it to SmartDashboard for selection.
                autoChooser = AutoBuilder.buildAutoChooser();
                SmartDashboard.putData("Auto Chooser", autoChooser);
                SmartDashboard.putData("Active Auto Path", autoField);
        }

        /**
         * Registers all the named commands that can be used in PathPlanner autonomous routines.
         * This allows complex actions to be triggered by name from the PathPlanner GUI.
         */
        private void registerNamedCommands() {
                // Command to score at level 1
                NamedCommands.registerCommand("scoreL1", new SequentialCommandGroup(
                                new InstantCommand(() -> pivot.changeSetpoint(PivotConstants.kElevatingSetpoint)),
                                new InstantCommand(() -> elevator.ChangeSetpoint(Elevator.kL1Setpoint)),
                                Commands.waitUntil(() -> elevator.atSetpoint(Elevator.kL1Setpoint))
                                                .andThen(() -> pivot.changeSetpoint(PivotConstants.kL1Setpoint)))
                                                                                .andThen(new RunCommand(
                                                                                                () -> roller.setRollerOutput(Roller.kEjectSpeed),
                                                                                                roller).until(() -> !roller.getHasCoral()).andThen(new InstantCommand(() -> roller.setRollerOutput(0.), roller))));

                // Command to score at level 2
                NamedCommands.registerCommand("scoreL2", new SequentialCommandGroup(
                                new InstantCommand(() -> pivot.changeSetpoint(PivotConstants.kElevatingSetpoint)),
                                new InstantCommand(() -> elevator.ChangeSetpoint(Elevator.kL2Setpoint)),
                                Commands.waitUntil(() -> elevator.atSetpoint(Elevator.kL2Setpoint))
                                                .andThen(() -> pivot.changeSetpoint(PivotConstants.kL2Setpoint)))
                                                                                .andThen(new RunCommand(
                                                                                                () -> roller.setRollerOutput(Roller.kEjectSpeed),
                                                                                                roller).until(() -> !roller.getHasCoral()).andThen(new InstantCommand(() -> roller.setRollerOutput(0.), roller))));

                // Command to score at level 3
                NamedCommands.registerCommand("scoreL3", new SequentialCommandGroup(
                                new InstantCommand(() -> pivot.changeSetpoint(PivotConstants.kElevatingSetpoint)),
                                new InstantCommand(() -> elevator.ChangeSetpoint(Elevator.kL3Setpoint)),
                                Commands.waitUntil(() -> elevator.atSetpoint(Elevator.kL3Setpoint))
                                                .andThen(() -> pivot.changeSetpoint(PivotConstants.kL3Setpoint)))
                                                                                .andThen(new RunCommand(
                                                                                                () -> roller.setRollerOutput(Roller.kEjectSpeed),
                                                                                                roller).until(() -> !roller.getHasCoral()).andThen(new InstantCommand(() -> roller.setRollerOutput(0.), roller))));

                // Command to score at level 4
                NamedCommands.registerCommand("scoreL4", new SequentialCommandGroup(
                                new InstantCommand(() -> pivot.changeSetpoint(PivotConstants.kElevatingSetpoint)),
                                new InstantCommand(() -> elevator.ChangeSetpoint(Elevator.kL4Setpoint)),
                                Commands.waitUntil(() -> elevator.atSetpoint(Elevator.kL4Setpoint))
                                                .andThen(() -> pivot.changeSetpoint(PivotConstants.kL4Setpoint)))
                                                                                .andThen(new RunCommand(
                                                                                                () -> roller.setRollerOutput(Roller.kEjectSpeed),
                                                                                                roller).until(() -> !roller.getHasCoral()).andThen(new InstantCommand(() -> roller.setRollerOutput(0.), roller))));

                // Command to indicate that the robot has reached a path target.
                NamedCommands.registerCommand("ReachedTarget", new InstantCommand(() -> autoGenerator.setReachedTarget(true)));

                // Command to reset the reached target flag.
                NamedCommands.registerCommand("ResetReachedTarget", new InstantCommand(() -> autoGenerator.setReachedTarget(false)));

                // Command to score at level 2, conditional on reaching a target.
                NamedCommands.registerCommand("scoreL2(conditional)", new SequentialCommandGroup(
                                Commands.waitUntil(() -> autoGenerator.getReachedTarget()),
                                new InstantCommand(() -> pivot.changeSetpoint(PivotConstants.kElevatingSetpoint)),
                                new InstantCommand(() -> elevator.ChangeSetpoint(Elevator.kL2Setpoint)),
                                Commands.waitUntil(() -> elevator.atSetpoint(Elevator.kL2Setpoint)),
                                new InstantCommand(() -> pivot.changeSetpoint(PivotConstants.kL2Setpoint)),
                                Commands.waitUntil(() -> pivot.atSetpoint(PivotConstants.kL2Setpoint)),
                                new RunCommand(() -> roller.setRollerOutput(Roller.kEjectSpeed - 0.1), roller).withTimeout(.15)));
                                                                
                // Command to score at level 4, conditional on reaching a target.
                NamedCommands.registerCommand("scoreL4(conditional)", new ParallelRaceGroup(new SequentialCommandGroup(
                                Commands.waitUntil(() -> autoGenerator.getReachedTarget()),
                                new InstantCommand(() -> pivot.changeSetpoint(PivotConstants.kElevatingSetpoint)),
                                new InstantCommand(() -> elevator.ChangeSetpoint(Elevator.kL4Setpoint)),
                                Commands.waitUntil(() -> elevator.atSetpoint(Elevator.kL4Setpoint)),
                                new InstantCommand(() -> pivot.changeSetpoint(PivotConstants.kL4Setpoint)),
                                Commands.waitUntil(() -> pivot.atSetpoint(PivotConstants.kL4Setpoint)),
                                new WaitCommand(.05),
                                new RunCommand(() -> roller.setRollerOutput(Roller.kEjectSpeed - 0.1), roller).withTimeout(.1)).withTimeout(4),
                                Commands.waitUntil(()->!autoGenerator.getIntakeComplete())));

                // Command to run the roller to eject game pieces.
                NamedCommands.registerCommand("runRoller", new RunCommand(() -> roller.setRollerOutput(Roller.kEjectSpeed, Roller.kRollerHelperSpeed), roller));

                // Command to intake a game piece.
                NamedCommands.registerCommand("intake", new ParallelRaceGroup(
                                new SequentialCommandGroup(
                                                new InstantCommand(() -> pivot.changeSetpoint(PivotConstants.kElevatingSetpoint)),
                                                new InstantCommand(() -> elevator.ChangeSetpoint(0.0)),
                                                Commands.waitUntil(() -> elevator.atSetpoint(0.0))
                                                                .andThen(() -> pivot.changeSetpoint(PivotConstants.kIntakeSetpoint)))
                                                                                .andThen(new RunCommand(() -> roller.setRollerOutput(Roller.kIntakeSpeed, Roller.kRollerHelperSpeed), roller)
                                                                                                .until(() -> roller.getHasCoral())
                                                                                                .andThen(new InstantCommand(() -> roller.setRollerOutput(0, 0))
                                                                                                .andThen(new InstantCommand(()->autoGenerator.setIntakeComplete(true))))),
                                new SequentialCommandGroup(new WaitCommand(4), new InstantCommand(()->autoGenerator.setIntakeComplete(false))))
                                                                                                );

                // Command to stow the intake mechanism to a safe position.
                NamedCommands.registerCommand("stow", new SequentialCommandGroup(
                                new InstantCommand(() -> pivot.changeSetpoint(PivotConstants.kElevatingSetpoint)),
                                new InstantCommand(() -> elevator.ChangeSetpoint(0.0)),
                                Commands.waitUntil(() -> elevator.atSetpoint(0.0))
                                                .andThen(() -> pivot.changeSetpoint(PivotConstants.kIntakeSetpoint))));

                // Command to move to the L2 algae intake position.
                NamedCommands.registerCommand("L2AlgaeIntake", getL2AlgaeSetpointCommand());

                // Command to grab algae.
                NamedCommands.registerCommand("grabAlgae",  new ParallelRaceGroup(
                                new InstantCommand(() -> pivot.changeSetpoint(PivotConstants.kElevatingSetpoint))
                                                .alongWith(new RunCommand(() -> roller.setRollerOutput(-Roller.kIntakeSpeed))),
                                                new WaitCommand(1.0)));
                
                
                // Command to move the elevator to the zero (bottom) position.
                NamedCommands.registerCommand("moveElevatorToZero", new SequentialCommandGroup(
                        new InstantCommand(() -> elevator.ChangeSetpoint(0.0)),
                        Commands.waitUntil(() -> elevator.atSetpoint(0.0))
                ));

                // Command to move the elevator to the processor and then score algae.
                NamedCommands.registerCommand("moveElevatorToProcessorAndScoreAlgae", new SequentialCommandGroup(
                        new InstantCommand(() -> elevator.ChangeSetpoint(Elevator.kProcessorSetpoint)),
                        Commands.waitUntil(() -> elevator.atSetpoint(Elevator.kProcessorSetpoint)),
                        new InstantCommand(() -> pivot.changeSetpoint(PivotConstants.kAlgaeScoringSetpoint)),
                        new RunCommand(() -> roller.setRollerOutput(-Roller.kEjectSpeed), roller).withTimeout(1.0)
                ));
        }

        /**
         * Use this method to define your trigger->command mappings. Triggers can be created via the
         * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
         * predicate, or via the named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s
         * subclasses for {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
         * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
         */
        private void configureBindings() {
                // --- Driver 1 Controls (Drivetrain and Alignment) --- //
                // Zero the gyro heading when the left stick is pressed.
                Driver1.leftStick().onTrue(new InstantCommand(() -> drivetrain.zeroHeading()));
                // Control the climber with the POV hat.
                Driver1.povUp().whileTrue(new RunCommand(() -> climber.setSpeed(Climber.kClimberPercentOutput))).onFalse(new InstantCommand(() -> climber.setSpeed(0.0)));
                Driver1.povDown().whileTrue(new RunCommand(() -> climber.setSpeed(-Climber.kClimberPercentOutput))).onFalse(new InstantCommand(() -> climber.setSpeed(0.0)));

                // Alignment commands
                Driver1.y().whileTrue(new CMD_PathfindAlgaeAlign(drivetrain, photonVision));
                Driver1.a().onTrue(new InstantCommand(() -> pivot.changeSetpoint(PivotConstants.kL2Setpoint)));
                Driver1.x().whileTrue(new CMD_PathfindReefAlign(drivetrain, photonVision, true, ()->targetId,()->listIndex));
                Driver1.b().whileTrue(new CMD_PathfindReefAlign(drivetrain, photonVision, false, ()->targetId,()->listIndex));
                Driver1.leftBumper().whileTrue(new CMD_OldPathfindReefAlign(drivetrain, photonVision, true)); // Right
                Driver1.leftTrigger().whileTrue(new CMD_OldPathfindReefAlign(drivetrain, photonVision, false)); // Left

                // Select the reef side with the right stick.
                Driver1.rightStick().onTrue(Commands.none()).onFalse(new InstantCommand(() -> getSelectedReefSide()));

                // Allow for driving without turning (strafe only) when the POV is used.
                Driver1.povLeft().whileTrue(new RunCommand(() -> drivetrain.drive(
                                MathUtil.applyDeadband(Driver1.getRawAxis(1), Operator.kDriveDeadband),
                                MathUtil.applyDeadband(Driver1.getRawAxis(0), Operator.kDriveDeadband),
                                0, true, true), drivetrain));
                Driver1.povUpLeft().whileTrue(new RunCommand(() -> drivetrain.drive(
                        MathUtil.applyDeadband(Driver1.getRawAxis(1), Operator.kDriveDeadband),
                        MathUtil.applyDeadband(Driver1.getRawAxis(0), Operator.kDriveDeadband),
                        0, true, true), drivetrain));
                Driver1.povDownLeft().whileTrue(new RunCommand(() -> drivetrain.drive(
                        MathUtil.applyDeadband(Driver1.getRawAxis(1), Operator.kDriveDeadband),
                        MathUtil.applyDeadband(Driver1.getRawAxis(0), Operator.kDriveDeadband),
                        0, true, true), drivetrain));

                // --- Driver 2 Controls (Scoring and Intake) --- //
                // Elevator and Pivot setpoints
                // Controls the buttons to move the elevator to predefined points in constants.java.
                // Notice how methods of classes are being called.
                Driver2.a().onTrue(getZeroSetpointCommand());
                Driver2.b().onTrue(getL2SetpointCommand());
                Driver2.x().onTrue(getL3SetpointCommand());
                Driver2.y().onTrue(getL4SetpointCommand());

                // Algae and special scoring setpoints
                Driver2.povUp().onTrue(getAlgaeSetpointCommand());
                Driver2.povDown().onTrue(getL2AlgaeSetpointCommand());
                Driver2.povLeft().onTrue(getProcessorSetpointCommand());
                Driver2.povRight().onTrue(getBargeScoringCommand());

                // Roller controls
                Driver2.leftBumper().whileTrue(new RunCommand(()->roller.setRollerOutput(-Roller.kIntakeSpeed, -Roller.kRollerHelperSpeed))).onFalse(new InstantCommand(()->roller.setRollerOutput(0.0, 0.0)));
                
                // Intake command with rumble feedback when a game piece is detected. We love rumble 😁😍
                Driver2.rightBumper().whileTrue(new RunCommand(() -> roller.setRollerOutput(Roller.kIntakeSpeed, Roller.kRollerHelperSpeed), roller)
                                .until(() -> roller.getHasCoral())
                                .andThen(new ParallelCommandGroup(
                                                new InstantCommand(() -> Driver1.getHID().setRumble(RumbleType.kBothRumble, 1)),
                                                new InstantCommand(() -> Driver2.getHID().setRumble(RumbleType.kBothRumble, 1)),
                                                new InstantCommand(() -> leds.set(LEDs.kColorGreen)),
                                                new RunCommand(() -> roller.setRollerOutput(Roller.kIntakeFinishSpeed, 0), roller))
                                                .withTimeout(Roller.kIntakeFinishTime)
                                                .andThen(new ParallelCommandGroup(
                                                                new InstantCommand(() -> Driver1.getHID().setRumble(RumbleType.kBothRumble, 0)),
                                                                new InstantCommand(() -> Driver2.getHID().setRumble(RumbleType.kBothRumble, 0))))))
                                .onFalse(new ParallelCommandGroup(
                                                new InstantCommand(() -> Driver1.getHID().setRumble(RumbleType.kBothRumble, 0)),
                                                new InstantCommand(() -> Driver2.getHID().setRumble(RumbleType.kBothRumble, 0))));

                // Eject command.
                Driver2.rightTrigger().whileTrue(new RunCommand(() -> roller.setRollerOutput(Roller.kEjectSpeed), roller)
                                .until(() -> !roller.getHasCoral())
                                .andThen(new SequentialCommandGroup(
                                                new InstantCommand(() -> roller.setRollerOutput(0.), roller),
                                                new InstantCommand(() -> leds.setAllianceColor()))))
                                .onFalse(new InstantCommand(() -> roller.setRollerOutput(0.), roller));

                // Algae scoring command.
                Driver2.leftTrigger().whileTrue(new InstantCommand(() -> pivot.changeSetpoint(PivotConstants.kAlgaeScoringSetpoint))
                                .alongWith(new RunCommand(() -> roller.setRollerOutput(0.95), roller)))
                                .onFalse(new InstantCommand(() -> roller.setRollerOutput(0.0), roller));
        }

        /**
         * Initializes robot-wide settings that are not subsystem-specific.
         */
        public void robotInit() {
                Pathfinding.setPathfinder(new LocalADStar());
                powerDistribution.setSwitchableChannel(true);
        }

        /**
         * Gets the selected reef side based on the driver's controller input.
         * This is used for dynamic alignment during teleop.
         */
        public void getSelectedReefSide() {
                double x = Driver1.getRawAxis(4);
                double y = -Driver1.getRawAxis(5);
                int[] targetTagSet = DriverStation.getAlliance().equals(Optional.of(Alliance.Red)) ? new int[]{10,11,6,7,8,9} : new int[]{21, 20, 19,18, 17, 22};
                double angleRadians;
                if (x==0 && y==0) {
                        angleRadians = 0.0;
                } else {
                        angleRadians = Math.atan2(y, x) - (Math.PI/2);
                }
                double angleDegrees = angleRadians*180/Math.PI;
                int reefAngleDegrees = (int)Math.round((angleDegrees)/60)*60;
                listIndex = Math.floorMod((int)Math.round((angleDegrees)/60),6);
                
                SmartDashboard.putNumber("Angle", angleDegrees);
                SmartDashboard.putNumber("Reef Side Angle", reefAngleDegrees);
                SmartDashboard.putNumber("Reef Align Target ID", targetTagSet[listIndex]);
                
                Pose2d pose = photonVision.at_field.getTagPose(targetId).orElse(new Pose3d()).toPose2d();
                drivetrain.publisher1.set(pose);
                targetId = targetTagSet[listIndex];
        }

        /**
         * Creates a pathfinding command for a given path name from the PathPlanner GUI.
         * @param pathName The name of the path file (without the .path extension).
         * @return A command that pathfinds to and follows the specified path.
         */
        public Command getPathCommand(String pathName) {
                Pathfinding.setPathfinder(new LocalADStar());
                try {
                        PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
                        PathConstraints constraints = new PathConstraints(0.5, 0.5,
                                        Units.degreesToRadians(180), Units.degreesToRadians(180));
                        return AutoBuilder.pathfindThenFollowPath(path, constraints);
                } catch (Exception e) {
                        DriverStation.reportError("Failed to load path: " + pathName + "; " + e.getMessage(), e.getStackTrace());
                        return Commands.none();
                }
        }

        // --- Command Factory Methods --- //

        /**
         * Returns a command to move the elevator and pivot to the processor setpoint.
         * @return A command to move to the processor setpoint.
         */
        public Command getProcessorSetpointCommand() {
                Command c = new ParallelRaceGroup(new SequentialCommandGroup(new InstantCommand(
                                () -> pivot.changeSetpoint(PivotConstants.kAlgaeSafeSetpoint)),
                                new InstantCommand(() -> elevator.ChangeSetpoint(Elevator.kProcessorSetpoint)),
                                Commands.waitUntil(() -> elevator.atSetpoint(Elevator.kProcessorSetpoint))
                                                .andThen(() -> pivot.changeSetpoint(PivotConstants.kAlgaeScoringSetpoint))),
                                new RunCommand(() -> elevator.runElevatorAlgae(() -> pivot.atSetpointAlgae(PivotConstants.kAlgaeSafeSetpoint))));
                c.addRequirements(elevator);
                return c;
        }

        /**
         * Returns a command to move the elevator and pivot to the barge setpoint.
         * @return A command to move to the barge setpoint.
         */
        public Command getBargeSetpointCommand() {
                Command c = new ParallelRaceGroup(new SequentialCommandGroup(new InstantCommand(
                                () -> pivot.changeSetpoint(PivotConstants.kAlgaeSafeSetpoint)),
                                new InstantCommand(() -> elevator.ChangeSetpoint(Elevator.kL4Setpoint)),
                                Commands.waitUntil(() -> elevator.atSetpoint(0.0))
                                                .andThen(() -> pivot.changeSetpoint(PivotConstants.kAlgaeScoringSetpoint))),
                                new RunCommand(() -> elevator.runElevatorAlgae(() -> pivot.atSetpoint(PivotConstants.kAlgaeSafeSetpoint))));
                c.addRequirements(elevator);
                return c;
        }

        /**
         * Returns a command to move the elevator and pivot to the L4 setpoint.
         * @return A command to move to the L4 setpoint.
         */
        public Command getL4SetpointCommand() {
                Command c = new ParallelRaceGroup(new SequentialCommandGroup(new InstantCommand(
                                () -> pivot.changeSetpoint(PivotConstants.kElevatingSetpoint)),
                                new InstantCommand(() -> elevator.ChangeSetpoint(Elevator.kL4Setpoint)),
                                Commands.waitUntil(() -> elevator.atSetpoint(Elevator.kL4Setpoint))
                                                .andThen(() -> pivot.changeSetpoint(PivotConstants.kL4Setpoint))),
                                new RunCommand(() -> elevator.runElevator(() -> pivot.atElevatingSetpoint())));
                c.addRequirements(elevator);
                return c;
        }

        /**
         * Returns a command to move the elevator and pivot to the L3 setpoint.
         * @return A command to move to the L3 setpoint.
         */
        public Command getL3SetpointCommand() {
                Command c = new ParallelRaceGroup(new SequentialCommandGroup(new InstantCommand(
                                () -> pivot.changeSetpoint(PivotConstants.kElevatingSetpoint)),
                                new InstantCommand(() -> elevator.ChangeSetpoint(Elevator.kL3Setpoint)),
                                Commands.waitUntil(() -> elevator.atSetpoint(Elevator.kL3Setpoint))
                                                .andThen(() -> pivot.changeSetpoint(PivotConstants.kL3Setpoint))),
                                new RunCommand(() -> elevator.runElevator(() -> pivot.atElevatingSetpoint())));
                c.addRequirements(elevator);
                return c;
        }

        /**
         * Returns a command to move the elevator and pivot to the L2 setpoint.
         * @return A command to move to the L2 setpoint.
         */
        public Command getL2SetpointCommand() {
                Command c = new ParallelRaceGroup(new SequentialCommandGroup(new InstantCommand(
                                () -> pivot.changeSetpoint(PivotConstants.kElevatingSetpoint)),
                                new InstantCommand(() -> elevator.ChangeSetpoint(Elevator.kL2Setpoint)),
                                Commands.waitUntil(() -> elevator.atSetpoint(Elevator.kL2Setpoint))
                                                .andThen(() -> pivot.changeSetpoint(PivotConstants.kL2Setpoint))),
                                new RunCommand(() -> elevator.runElevator(() -> pivot.atElevatingSetpoint())));
                c.addRequirements(elevator);
                return c;
        }

        /**
         * Returns a command for scoring on the barge.
         * @return A command for scoring on the barge.
         */
        public Command getBargeScoringCommand() {
                Command c = new ParallelRaceGroup(new SequentialCommandGroup(new InstantCommand(
                                () -> pivot.changeSetpoint(PivotConstants.kAlgaeSafeSetpoint)),
                                new InstantCommand(() -> elevator.ChangeSetpoint(Elevator.kL4Setpoint)),
                                Commands.waitUntil(()->elevator.getCurrentPosition() > Elevator.kL2Setpoint),
                                new InstantCommand(()->pivot.changeSetpoint(103)),
                                Commands.waitUntil(() -> elevator.atSetpoint(.69)),
                                new InstantCommand(() -> pivot.changeSetpoint(240)),
                                new ParallelRaceGroup(new SequentialCommandGroup(
                                                Commands.waitUntil(() -> pivot.atSetpoint(231.5)),
                                                new RunCommand(() -> roller.setRollerOutput(-Roller.kIntakeSpeed))),
                                                new SequentialCommandGroup(Commands.waitUntil(() -> pivot.atSetpoint(240)), new WaitCommand(.2)),
                                                new InstantCommand(() -> pivot.changeSetpoint(PivotConstants.kAlgaeSafeSetpoint)),
                                                new InstantCommand(() -> elevator.ChangeSetpoint(0.0)))),
                                new RunCommand(() -> elevator.runElevatorAlgae(() -> pivot.atSetpointAlgae(PivotConstants.kAlgaeSafeSetpoint))));
                c.addRequirements(elevator);
                return c;
        }

        /**
         * Returns a command to move the elevator and pivot to the zero (intake) setpoint.
         * @return A command to move to the zero setpoint.
         */
        public Command getZeroSetpointCommand() {
                Command c = new ParallelRaceGroup(new SequentialCommandGroup(new InstantCommand(
                                () -> pivot.changeSetpoint(PivotConstants.kElevatingSetpoint)),
                                new InstantCommand(() -> elevator.ChangeSetpoint(0.0)),
                                Commands.waitUntil(() -> elevator.atSetpoint(0.0))
                                                .andThen(() -> pivot.changeSetpoint(PivotConstants.kIntakeSetpoint))),
                                new RunCommand(() -> elevator.runElevator(() -> pivot.atElevatingSetpoint())));
                c.addRequirements(elevator);
                return c;
        }

        /**
         * Returns a command to move the elevator and pivot to the algae intake setpoint.
         * @return A command to move to the algae intake setpoint.
         */
        public Command getAlgaeSetpointCommand() {
                Command c = new ParallelRaceGroup(new SequentialCommandGroup(new InstantCommand(
                                () -> pivot.changeSetpoint(PivotConstants.kElevatingSetpoint)),
                                new InstantCommand(() -> elevator.ChangeSetpoint(Elevator.kAlgaeSetpoint + 0.05)),
                                Commands.waitUntil(() -> elevator.atSetpoint(Elevator.kAlgaeSetpoint + 0.05))
                                                .andThen(() -> pivot.changeSetpoint(PivotConstants.kAlgaeSetpoint))),
                                new RunCommand(() -> elevator.runElevator(() -> pivot.atSetpoint(PivotConstants.kElevatingSetpoint))));
                c.addRequirements(elevator);
                return c;
        }

        /**
         * Returns a command to move the elevator and pivot to the de-algae setpoint.
         * @return A command to move to the de-algae setpoint.
         */
        public Command getDealgaeSetpointCommand() {
                Command c = new ParallelRaceGroup(new SequentialCommandGroup(new InstantCommand(
                                () -> pivot.changeSetpoint(PivotConstants.kElevatingSetpoint)),
                                new InstantCommand(() -> elevator.ChangeSetpoint(Elevator.kAlgaeSetpoint)),
                                Commands.waitUntil(() -> elevator.atSetpoint(Elevator.kAlgaeSetpoint))
                                                .andThen(() -> pivot.changeSetpoint(PivotConstants.kAlgaeSetpoint))),
                                new RunCommand(() -> elevator.runElevator(() -> pivot.atSetpoint(PivotConstants.kElevatingSetpoint))));
                c.addRequirements(elevator);
                return c;
        }

        /**
         * Returns a command to move the elevator and pivot to the L2 algae setpoint.
         * @return A command to move to the L2 algae setpoint.
         */
        public Command getL2AlgaeSetpointCommand() {
                Command c = new ParallelRaceGroup(new SequentialCommandGroup(new InstantCommand(
                                () -> pivot.changeSetpoint(PivotConstants.kElevatingSetpoint)),
                                new InstantCommand(() -> elevator.ChangeSetpoint(Elevator.kL2Setpoint + 0.1)),
                                Commands.waitUntil(() -> elevator.atSetpoint(Elevator.kL2Setpoint + 0.1))
                                                .andThen(() -> pivot.changeSetpoint(PivotConstants.kAlgaeSetpoint))),
                                new RunCommand(() -> elevator.runElevator(() -> pivot.atSetpoint(PivotConstants.kElevatingSetpoint))));
                c.addRequirements(elevator);
                return c;
        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }

        /**
         * This method is called periodically across all robot modes.
         * It updates SmartDashboard with battery voltage, match time, and the robot's pose.
         */
        public void robotPeriodic() {
                SmartDashboard.putNumber("Battery Voltage", powerDistribution.getVoltage());
                SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
                autoField.setRobotPose(drivetrain.getPose());
        }

        /**
         * This method is called once when autonomous is initialized.
         * It sets initial states, selects the "Autonomous" tab on Elastic dashboard, and configures PathPlanner logging.
         */
        public void autonomousInit() {
                autoGenerator.setIntakeComplete(true);
                autoGenerator.setReachedTarget(false);
                Elastic.selectTab("Autonomous");
                leds.set(LEDs.kParty_Palette_Twinkles);
                PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
                        Pose2d currentPose = drivetrain.getPose();
                        SmartDashboard.putNumber("X Error", pose.getX() - currentPose.getX());
                        SmartDashboard.putNumber("Y Error", pose.getY() - currentPose.getY());
                        SmartDashboard.putNumber("Theta Error", pose.getRotation().getRadians() - currentPose.getRotation().getRadians());
                        SmartDashboard.putNumber("Desired Theta", pose.getRotation().getRadians());
                        SmartDashboard.putNumber("Actual Theta", currentPose.getRotation().getRadians());
                });
        }

        /**
         * This method is called periodically during autonomous.
         * It updates the robot's pose estimate using PhotonVision data.
         */
        public void autonomousPeriodic() {
                photonAutonPoseUpdate();
        }

        /**
         * This method is called once when teleop is initialized.
         * It sets the LEDs to the alliance color and selects the "Teleoperated" tab on Elastic dashboard.
         */
        public void teleopInit() {
                leds.setAllianceColor();
                Elastic.selectTab("Teleoperated");
        }

        /**
         * This method is called periodically during teleoperated mode.
         * It updates the robot's pose estimate using PhotonVision data.
         */
        public void teleopPeriodic() {
                photonPoseUpdate();
        }

        /**
         * This method is called periodically while the robot is disabled.
         * It updates the autonomous path visualization on SmartDashboard when the selected auto or alliance changes.
         */
        public void disabledPeriodic() {
                newAutoName = getAutonomousCommand().getName();
                alliance = DriverStation.getAlliance();
                if (autoName != newAutoName || alliance != lastAlliance) {
                        autoName = newAutoName;
                        lastAlliance = alliance;
                        if (AutoBuilder.getAllAutoNames().contains(autoName)) {
                                try {
                                        List<PathPlannerPath> pathPlannerPaths = PathPlannerAuto.getPathGroupFromAutoFile(autoName);
                                        List<Pose2d> poses = new ArrayList<>();
                                        for (PathPlannerPath path : pathPlannerPaths) {
                                                if (DriverStation.getAlliance().equals(Optional.of(Alliance.Red))) {
                                                        poses.addAll(path.getAllPathPoints().stream()
                                                                        .map(point -> new Pose2d(
                                                                                        Field.fieldLength - point.position.getX(),
                                                                                        Field.fieldWidth - point.position.getY(),
                                                                                        new Rotation2d()))
                                                                        .collect(Collectors.toList()));
                                                } else {
                                                        poses.addAll(path.getAllPathPoints().stream()
                                                                        .map(point -> new Pose2d(
                                                                                        point.position.getX(),
                                                                                        point.position.getY(),
                                                                                        new Rotation2d()))
                                                                        .collect(Collectors.toList()));
                                                }
                                        }
                                        autoField.getObject("path").setPoses(poses);
                                } catch (IOException | ParseException e) {
                                        e.printStackTrace();
                                        return;
                                }
                        }
                }
                photonPoseUpdate();
        }

        /**
         * Updates the robot's pose estimate using data from both PhotonVision cameras.
         * It calculates a standard deviation for the vision measurement based on the distance to the target
         * and fuses it with the drivetrain's odometry.
         */
        public static void photonPoseUpdate() {
                Optional<EstimatedRobotPose> photonPoseOptional = photonVision.getCam1Pose();
                if (photonPoseOptional.isPresent()) {
                        Pose3d photonPose = photonPoseOptional.get().estimatedPose;
                        if (photonPose.getX() >= 0 && photonPose.getX() <= Field.fieldLength
                                        && photonPose.getY() >= 0 && photonPose.getY() <= Field.fieldWidth
                                        && photonVision.getCam1BestTarget() != null) {
                                Pose2d closestTag = photonVision.at_field.getTagPose(photonVision.getCam1BestTarget().getFiducialId()).get().toPose2d();
                                Translation2d translate = closestTag.minus(photonPose.toPose2d()).getTranslation();
                                double distance = translate.getNorm();
                                double xStddev = Math.pow(distance, 2) / (8.0088 * 0.5);
                                double yStddev = xStddev;
                                double rotStddev = Units.degreesToRadians(120.0);
                                drivetrain.publisher3.set(photonPose.toPose2d());
                                drivetrain.m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(xStddev, yStddev, rotStddev));
                                drivetrain.addVisionMeasurement(photonPose.toPose2d(), photonPoseOptional.get().timestampSeconds);
                        }
                }

                photonPoseOptional = photonVision.getCam2Pose();
                if (photonPoseOptional.isPresent()) {
                        Pose3d photonPose = photonPoseOptional.get().estimatedPose;
                        if (photonPose.getX() >= 0 && photonPose.getX() <= Field.fieldLength
                                        && photonPose.getY() >= 0 && photonPose.getY() <= Field.fieldWidth
                                        && photonVision.getCam2BestTarget() != null) {
                                Pose2d closestTag = photonVision.at_field.getTagPose(photonVision.getCam2BestTarget().getFiducialId()).get().toPose2d();
                                Translation2d translate = closestTag.minus(photonPose.toPose2d()).getTranslation();
                                double distance = translate.getNorm();
                                double xStddev = Math.pow(distance, 2) / 8.0088;
                                double yStddev = xStddev;
                                double rotStddev = Units.degreesToRadians(120.0);
                                drivetrain.publisher4.set(photonPose.toPose2d());
                                drivetrain.m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(xStddev, yStddev, rotStddev));
                                drivetrain.addVisionMeasurement(photonPose.toPose2d(), photonPoseOptional.get().timestampSeconds);
                        }
                }
        }

        /**
         * Updates the robot's pose estimate using PhotonVision data during autonomous mode.
         * This version uses a different standard deviation calculation that incorporates the robot's current speed.
         */
        public static void photonAutonPoseUpdate() {
                Optional<EstimatedRobotPose> photonPoseOptional = photonVision.getCam1Pose();
                if (photonPoseOptional.isPresent()) {
                        Pose3d photonPose = photonPoseOptional.get().estimatedPose;
                        if (photonPose.getX() >= 0 && photonPose.getX() <= Field.fieldLength
                                        && photonPose.getY() >= 0 && photonPose.getY() <= Field.fieldWidth
                                        && photonVision.getCam1BestTarget() != null) {
                                Pose2d closestTag = photonVision.at_field.getTagPose(photonVision.getCam1BestTarget().getFiducialId()).get().toPose2d();
                                Translation2d translate = closestTag.minus(photonPose.toPose2d()).getTranslation();
                                double distance = translate.getNorm();
                                double xStddev = Math.pow(distance, 1.75) * (3 * (Math.sqrt(Math.pow(drivetrain.getChassisSpeeds().vxMetersPerSecond,2)+Math.pow(drivetrain.getChassisSpeeds().vyMetersPerSecond,2)))/ 4.92) / 3.6;
                                double yStddev = xStddev;
                                double rotStddev = Units.degreesToRadians(120.0);
                                drivetrain.publisher3.set(photonPose.toPose2d());
                                drivetrain.m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(xStddev, yStddev, rotStddev));
                                drivetrain.addVisionMeasurement(photonPose.toPose2d(), photonPoseOptional.get().timestampSeconds);
                        }
                }

                photonPoseOptional = photonVision.getCam2Pose();
                if (photonPoseOptional.isPresent()) {
                        Pose3d photonPose = photonPoseOptional.get().estimatedPose;
                        if (photonPose.getX() >= 0 && photonPose.getX() <= Field.fieldLength
                                        && photonPose.getY() >= 0 && photonPose.getY() <= Field.fieldWidth
                                        && photonVision.getCam2BestTarget() != null) {
                                Pose2d closestTag = photonVision.at_field.getTagPose(photonVision.getCam2BestTarget().getFiducialId()).get().toPose2d();
                                Translation2d translate = closestTag.minus(photonPose.toPose2d()).getTranslation();
                                double distance = translate.getNorm();
                                double xStddev = Math.pow(distance, 1.75) * (3 * (Math.sqrt(Math.pow(drivetrain.getChassisSpeeds().vxMetersPerSecond,2)+Math.pow(drivetrain.getChassisSpeeds().vyMetersPerSecond,2)))/ 4.92) / 3.6;
                                double yStddev = xStddev;
                                double rotStddev = Units.degreesToRadians(120.0);
                                drivetrain.publisher4.set(photonPose.toPose2d());
                                drivetrain.m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(xStddev, yStddev, rotStddev));
                                drivetrain.addVisionMeasurement(photonPose.toPose2d(), photonPoseOptional.get().timestampSeconds);
                        }
                }
        }
}
