// MONKEY
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
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Climber;
import frc.robot.Constants.Elevator;
import frc.robot.Constants.Field;
import frc.robot.Constants.LEDs;
import frc.robot.Constants.Operator;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.Roller;
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
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
        // The robot's subsystems and commands are defined here...
        private static final SUB_Drivetrain drivetrain = SUB_Drivetrain.getInstance();
        private static final SUB_PhotonVision photonVision = SUB_PhotonVision.getInstance();
        private static final AutoGenerator autoGenerator = AutoGenerator.getInstance();
        private final SendableChooser<Command> autoChooser;
        public static SUB_Elevator elevator = SUB_Elevator.getInstance();
        public static SUB_Roller roller = SUB_Roller.getInstance();
        public static SUB_Pivot pivot = SUB_Pivot.getInstance(roller.getAbsoluteEncoder());
        public static SUB_Climber climber = SUB_Climber.getInstance();
        public static SUB_LEDs leds = SUB_LEDs.getInstance();
        public static PowerDistribution powerDistribution = new PowerDistribution();
        private static String autoName, newAutoName;
        Optional<Alliance> lastAlliance;
        Optional<Alliance> alliance;
        public static Field2d autoField = new Field2d();
        public boolean LStickPressed = false;

        public int targetId = 7;

        // Replace with CommandPS4Controller or CommandJoystick if needed
        private final CommandXboxController Driver1 = new CommandXboxController(Operator.kDriver1ControllerPort);

        private final CommandXboxController Driver2 = new CommandXboxController(Operator.kDriver2ControllerPort);

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                drivetrain.setDefaultCommand(new RunCommand( // Unstable
                                () -> drivetrain.drive(
                                                MathUtil.applyDeadband(Driver1.getRawAxis(1), Operator.kDriveDeadband),
                                                MathUtil.applyDeadband(Driver1.getRawAxis(0), Operator.kDriveDeadband),
                                                -MathUtil.applyDeadband(Driver1.getRawAxis(4), Operator.kDriveDeadband),
                                                true, true),
                                drivetrain));

                // Trigger c = new
                // Trigger(()->!pivot.atSetpoint(PivotConstants.kElevatingSetpoint))
                // c.onTrue(new RunCommand(() -> elevator.runElevator(), elevator);

                elevator.setDefaultCommand(new RunCommand(
                                () -> elevator
                                                .runElevator(() -> pivot.atSetpoint(PivotConstants.kElevatingSetpoint)),
                                elevator));
                pivot.setDefaultCommand(
                                new RunCommand(() -> pivot.runPivot(() -> roller.getHasCoral()), pivot));
                roller.setDefaultCommand(new RunCommand(() -> roller.setRollerOutput(0.0), roller));

                Driver1.rightBumper()
                                .whileTrue(
                                                new RunCommand(() -> drivetrain.drive(
                                                                -MathUtil.applyDeadband(
                                                                                Math.copySign(Math.pow(
                                                                                                Driver1.getRawAxis(1),
                                                                                                2),
                                                                                                Driver1
                                                                                                                .getRawAxis(1)),
                                                                                Operator.kDriveDeadband),
                                                                -MathUtil
                                                                                .applyDeadband(
                                                                                                Math.copySign(Math.pow(
                                                                                                                Driver1.getRawAxis(
                                                                                                                                0),
                                                                                                                2),
                                                                                                                Driver1.getRawAxis(
                                                                                                                                0)),
                                                                                                Operator.kDriveDeadband),
                                                                -MathUtil.applyDeadband(Driver1.getRawAxis(4),
                                                                                Operator.kDriveDeadband),
                                                                false, true), drivetrain));

                // File pathFolder = new File(Filesystem.getDeployDirectory() +
                // "/pathplanner/paths/");
                // File[] listOfFiles = pathFolder.listFiles();
                // List<String> pathNames = new ArrayList<>();

                // if (listOfFiles != null) {
                // for (File file : listOfFiles) {
                // if (file.isFile() && file.getName().endsWith(".path")) {
                // pathNames.add(file.getName());
                // }
                // }
                // }

                // for (String pathName : pathNames) {
                // String modifiedPathName = pathName.substring(0, pathName.length() - 5);
                // NamedCommands.registerCommand(modifiedPathName + " Pathfind",
                // getPathCommand(modifiedPathName));
                // }

                NamedCommands.registerCommand("scoreL1", new SequentialCommandGroup(
                                new InstantCommand(() -> pivot.changeSetpoint(PivotConstants.kElevatingSetpoint)),
                                new InstantCommand(() -> elevator.ChangeSetpoint(Elevator.kL1Setpoint)),
                                Commands.waitUntil(() -> elevator.atSetpoint(Elevator.kL1Setpoint))
                                                .andThen(() -> pivot.changeSetpoint(PivotConstants.kL1Setpoint)))
                                .andThen(
                                                new RunCommand(() -> roller.setRollerOutput(Roller.kEjectSpeed),
                                                                roller).until(() -> !roller.getHasCoral()).andThen(
                                                                                new InstantCommand(() -> roller
                                                                                                .setRollerOutput(0.),
                                                                                                roller))));

                NamedCommands.registerCommand("scoreL2", new SequentialCommandGroup(
                                new InstantCommand(() -> pivot.changeSetpoint(PivotConstants.kElevatingSetpoint)),
                                new InstantCommand(() -> elevator.ChangeSetpoint(Elevator.kL2Setpoint)),
                                Commands.waitUntil(() -> elevator.atSetpoint(Elevator.kL2Setpoint))
                                                .andThen(() -> pivot.changeSetpoint(PivotConstants.kL2Setpoint)))
                                .andThen(
                                                new RunCommand(() -> roller.setRollerOutput(Roller.kEjectSpeed),
                                                                roller).until(() -> !roller.getHasCoral()).andThen(
                                                                                new InstantCommand(() -> roller
                                                                                                .setRollerOutput(0.),
                                                                                                roller))));

                NamedCommands.registerCommand("scoreL3", new SequentialCommandGroup(
                                new InstantCommand(() -> pivot.changeSetpoint(PivotConstants.kElevatingSetpoint)),
                                new InstantCommand(() -> elevator.ChangeSetpoint(Elevator.kL3Setpoint)),
                                Commands.waitUntil(() -> elevator.atSetpoint(Elevator.kL3Setpoint))
                                                .andThen(() -> pivot.changeSetpoint(PivotConstants.kL3Setpoint)))
                                .andThen(
                                                new RunCommand(() -> roller.setRollerOutput(Roller.kEjectSpeed),
                                                                roller).until(() -> !roller.getHasCoral()).andThen(
                                                                                new InstantCommand(() -> roller
                                                                                                .setRollerOutput(0.),
                                                                                                roller))));

                NamedCommands.registerCommand("scoreL4", new SequentialCommandGroup(
                                new InstantCommand(() -> pivot.changeSetpoint(PivotConstants.kElevatingSetpoint)),
                                new InstantCommand(() -> elevator.ChangeSetpoint(Elevator.kL4Setpoint)),
                                Commands.waitUntil(() -> elevator.atSetpoint(Elevator.kL4Setpoint))
                                                .andThen(() -> pivot.changeSetpoint(PivotConstants.kL4Setpoint)))
                                .andThen(
                                                new RunCommand(() -> roller.setRollerOutput(Roller.kEjectSpeed),
                                                                roller).until(() -> !roller.getHasCoral()).andThen(
                                                                                new InstantCommand(() -> roller
                                                                                                .setRollerOutput(0.),
                                                                                                roller))));

                NamedCommands.registerCommand("scoreL4(nostop)", new SequentialCommandGroup(
                                new InstantCommand(() -> pivot.changeSetpoint(PivotConstants.kElevatingSetpoint)),
                                new InstantCommand(() -> elevator.ChangeSetpoint(Elevator.kL4Setpoint)),
                                Commands.waitUntil(() -> elevator.atSetpoint(Elevator.kL4Setpoint))
                                                .andThen(() -> pivot.changeSetpoint(PivotConstants.kL4Setpoint)))
                                .andThen(
                                                new RunCommand(() -> roller.setRollerOutput(Roller.kEjectSpeed),
                                                                roller)));

                NamedCommands.registerCommand("ReachedTarget", new InstantCommand(

                                () -> autoGenerator.setreachedtarget(true)));

                NamedCommands.registerCommand("ResetReachedTarget",
                                new InstantCommand(() -> autoGenerator.setreachedtarget(false)));

                NamedCommands.registerCommand("scoreL4(conditional)",
                                new SequentialCommandGroup(Commands.waitUntil(() -> autoGenerator.getreachedtarget()),
                                                new InstantCommand(
                                                                () -> pivot.changeSetpoint(
                                                                                PivotConstants.kElevatingSetpoint)),
                                                new InstantCommand(
                                                                () -> elevator.ChangeSetpoint(Elevator.kL4Setpoint)),
                                                Commands.waitUntil(() -> elevator.atSetpoint(Elevator.kL4Setpoint)),
                                                new InstantCommand(() -> pivot.changeSetpoint(
                                                                                PivotConstants.kL4Setpoint)),
                                                new RunCommand(() -> roller.setRollerOutput(Roller.kEjectSpeed),roller)
                                                .withTimeout(.15)));

                NamedCommands.registerCommand("runRoller",
                                new RunCommand(() -> roller.setRollerOutput(Roller.kEjectSpeed), roller));

                NamedCommands.registerCommand("intake", new SequentialCommandGroup(
                                new InstantCommand(() -> pivot.changeSetpoint(PivotConstants.kElevatingSetpoint)),
                                new InstantCommand(() -> elevator.ChangeSetpoint(0.0)),
                                Commands.waitUntil(() -> elevator.atSetpoint(0.0))

                                                .andThen(() -> pivot.changeSetpoint(PivotConstants.kIntakeSetpoint)))
                                .andThen(new RunCommand(
                                                () -> roller.setRollerOutput(Roller.kIntakeSpeed), roller)
                                                .until(() -> roller.getHasCoral())
                                                .andThen(new InstantCommand(
                                                                () -> roller.setRollerOutput(0)))))
                                                                ;

                NamedCommands.registerCommand("stow", new SequentialCommandGroup(
                                new InstantCommand(() -> pivot.changeSetpoint(PivotConstants.kElevatingSetpoint)),
                                new InstantCommand(() -> elevator.ChangeSetpoint(0.0)),
                                Commands.waitUntil(() -> elevator.atSetpoint(0.0))
                                                .andThen(() -> pivot.changeSetpoint(PivotConstants.kIntakeSetpoint))));

                // Configure the trigger bindings
                configureBindings();

                autoChooser = AutoBuilder.buildAutoChooser();
                SmartDashboard.putData("Auto Chooser", autoChooser);
                SmartDashboard.putData("Active Auto Path", autoField);

        }

        /**
         * Use this method to define your trigger->command mappings. Triggers can be
         * created via the
         * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
         * an arbitrary
         * predicate, or via the named factories in
         * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
         * for
         * {@link CommandXboxController
         * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
         * controllers or
         * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
         * joysticks}.
         */
        private void configureBindings() {

                Driver1.leftStick().onTrue(new InstantCommand(() -> drivetrain.zeroHeading())); // TODO:
                                                                                                // Change
                Driver1.leftTrigger()
                                .whileTrue(new RunCommand(() -> climber.setSpeed(Climber.kClimberPercentOutput)))
                                .onFalse(new InstantCommand(() -> climber.setSpeed(0.0)));
                Driver1.leftBumper()
                                .whileTrue(new RunCommand(() -> climber.setSpeed(-Climber.kClimberPercentOutput)))
                                .onFalse(new InstantCommand(() -> climber.setSpeed(0.0)));

                Driver1.y().onTrue(
                                new InstantCommand(() -> pivot.changeSetpoint(PivotConstants.kIntakeSetpoint)));
                Driver1.a().onTrue(
                                new InstantCommand(() -> pivot.changeSetpoint(PivotConstants.kAlgaeSetpoint)));

                Driver1.x().whileTrue(new CMD_PathfindReefAlign(drivetrain, photonVision, true, targetId));
                Driver1.b().whileTrue(new CMD_PathfindReefAlign(drivetrain, photonVision, false, targetId));
                Driver1.leftStick().whileTrue(new SequentialCommandGroup(
                        new InstantCommand(() -> Driver1LeftStickPressed(true)),
                        new RunCommand( // Unstable
                                () -> drivetrain.drive(
                                                MathUtil.applyDeadband(Driver1.getRawAxis(1), Operator.kDriveDeadband),
                                                MathUtil.applyDeadband(Driver1.getRawAxis(0), Operator.kDriveDeadband),
                                                0*-MathUtil.applyDeadband(Driver1.getRawAxis(4), Operator.kDriveDeadband),
                                                true, true),
                                drivetrain)))
                                .onFalse(new InstantCommand(() -> Driver1LeftStickPressed(false)));
                Driver1.rightStick().onTrue(new ConditionalCommand(new InstantCommand(() -> getSelectedReefSide()),Commands.none(),() -> LStickPressed));
                // Driver 2

                Driver2.a().onTrue(getZeroSetpointCommand());

                Driver2.b().onTrue(getL2SetpointCommand());

                Driver2.x().onTrue(getL3SetpointCommand());

                Driver2.y().onTrue(getL4SetpointCommand());

                Driver2.povUp().onTrue(getAlgaeSetpointCommand());
                Driver2.povDown().onTrue(getL2AlgaeSetpointCommand());
                Driver2.povLeft().onTrue(getProcessorSetpointCommand());

                
                // Driver2.povDown().onTrue(new InstantCommand(() ->
                // pivot.changeVoltage(-0.02)));
                // Driver2.povUp().onTrue(new InstantCommand(() -> pivot.changeVoltage(0.02)));

                // Driver2.leftBumper()
                // .whileTrue(new InstantCommand(() -> roller.timerInteract(true))
                // .andThen(new RunCommand(
                // () -> roller.setRollerOutput(Roller.kIntakeSpeed),
                // roller).until(() -> roller.atCurrentThresholdandTimerElapsed()))
                // .andThen(new ParallelCommandGroup(
                // new RunCommand(() -> roller.setRollerOutput(
                // Roller.kIntakeFinishSpeed), roller),
                // new InstantCommand(
                // () -> roller.timerInteract(false)),
                // new InstantCommand(() -> Driver1.getHID().setRumble(
                // RumbleType.kBothRumble, 1)),
                // new InstantCommand(() -> Driver2.getHID().setRumble(
                // RumbleType.kBothRumble, 1)),
                // new InstantCommand(() -> roller.hasCoral(true)))
                // .withTimeout(Roller.kIntakeFinishTime)
                // .andThen(new ParallelCommandGroup(
                // new InstantCommand(
                // () -> Driver1.getHID()
                // .setRumble(RumbleType.kBothRumble,
                // 0)),
                // new InstantCommand(
                // () -> Driver2.getHID()
                // .setRumble(RumbleType.kBothRumble,
                // 0)))))
                // .andThen(new InstantCommand(() -> roller.setRollerOutput(0.),
                // roller)))
                // .onFalse(new InstantCommand(() -> roller.setRollerOutput(0.),
                // roller));

                Driver2.rightBumper()
                                .whileTrue(new RunCommand(() -> roller.setRollerOutput(Roller.kIntakeSpeed), roller)
                                                .until(() -> roller.getHasCoral())

                                                .andThen(new ParallelCommandGroup(new InstantCommand(
                                                                () -> Driver1.getHID().setRumble(RumbleType.kBothRumble,
                                                                                1)),
                                                                new InstantCommand(() -> Driver2.getHID()
                                                                                .setRumble(RumbleType.kBothRumble, 1)),
                                                                new InstantCommand(() -> leds.set(LEDs.kColorGreen)),
                                                                new RunCommand(
                                                                                () -> roller.setRollerOutput(
                                                                                                Roller.kIntakeFinishSpeed),
                                                                                roller))
                                                                .withTimeout(Roller.kIntakeFinishTime)
                                                                .andThen(new ParallelCommandGroup(
                                                                                new InstantCommand(
                                                                                                () -> Driver1.getHID()
                                                                                                                .setRumble(
                                                                                                                                RumbleType.kBothRumble,
                                                                                                                                0)),
                                                                                new InstantCommand(() -> Driver2
                                                                                                .getHID()
                                                                                                .setRumble(RumbleType.kBothRumble,
                                                                                                                0))))))
                                .onFalse(new ParallelCommandGroup(
                                                new InstantCommand(
                                                                () -> Driver1.getHID().setRumble(RumbleType.kBothRumble,
                                                                                0)),
                                                new InstantCommand(
                                                                () -> Driver2.getHID().setRumble(RumbleType.kBothRumble,
                                                                                0))));

                Driver2.rightTrigger()
                                .whileTrue(new RunCommand(() -> roller.setRollerOutput(Roller.kEjectSpeed), roller)
                                                .until(() -> !roller.getHasCoral())
                                                .andThen(new SequentialCommandGroup(
                                                                new InstantCommand(() -> roller.setRollerOutput(0.),
                                                                                roller),
                                                                new InstantCommand(() -> leds.setAllianceColor()))))
                                .onFalse(new InstantCommand(() -> roller.setRollerOutput(0.), roller));

                // Driver2.leftBumper()
                // .whileTrue(new RunCommand(() -> roller.setRollerOutput(-Roller.kIntakeSpeed),
                // roller)
                // .andThen(Commands.waitSeconds(1)).andThen(new InstantCommand(() -> pivot
                // .changeSetpoint(PivotConstants.kElevatingSetpoint))))
                // .onFalse(new InstantCommand(() -> roller.setRollerOutput(0), roller));

                Driver2.leftBumper()
                                .whileTrue(new InstantCommand(
                                                () -> pivot.changeSetpoint(PivotConstants.kElevatingSetpoint))
                                                .alongWith(
                                                                new RunCommand(() -> roller.setRollerOutput(
                                                                                -Roller.kIntakeSpeed))))
                                .onFalse(new InstantCommand(() -> roller.setRollerOutput(0), roller));

                Driver2.leftTrigger().whileTrue(
                                new InstantCommand(() -> pivot.changeSetpoint(PivotConstants.kAlgaeScoringSetpoint))
                                                .alongWith(new RunCommand(() -> roller.setRollerOutput(0.95), roller)))
                                .onFalse(new InstantCommand(() -> roller.setRollerOutput(0.0), roller));

        }

        public void robotInit() {
                Pathfinding.setPathfinder(new LocalADStar());
                powerDistribution.setSwitchableChannel(true);
        }

        public void getSelectedReefSide() {
                List<Integer> targetTagSet = alliance.get() == DriverStation.Alliance.Red ? Arrays.asList(7, 8, 9, 10, 11, 6) : Arrays.asList(21, 20, 19, 18, 17, 22);
                double x = Driver1.getRawAxis(4);
                double y = Driver1.getRawAxis(5);
                if (x==0 && y==0) {
                        targetId = targetTagSet.indexOf(0);
                }
                double angleRadians = Math.atan2(y, x) + (x < 0 ? Math.PI : 0);
                double angleDegrees = Math.toDegrees(angleRadians);
                int reefAngleDegrees = (int)Math.round((angleDegrees-90)/60)*60;
                int listIndex = Math.floorMod((int)Math.round((angleDegrees-90)/60),6);
                
                SmartDashboard.putNumber("Angle", Math.toDegrees(angleRadians));
                SmartDashboard.putNumber("Reef Side Angle", reefAngleDegrees);
                SmartDashboard.putNumber("Reef Align Target ID", targetTagSet.indexOf(listIndex));
                
                Pose2d pose = photonVision.at_field.getTagPose(targetId).orElse(new Pose3d()).toPose2d();
                drivetrain.publisher1.set(pose);
                targetId = targetTagSet.indexOf(listIndex);
        }
        public void Driver1LeftStickPressed(boolean pressed) {
                LStickPressed = pressed;
        }
        public Command getPathCommand(String pathName) {
                Pathfinding.setPathfinder(new LocalADStar());
                try {
                        PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
                        PathConstraints constraints = new PathConstraints(0.5, 0.5, Units.degreesToRadians(180),
                                        Units.degreesToRadians(180)); // unstable
                        return AutoBuilder.pathfindThenFollowPath(path, constraints);
                } catch (Exception e) {
                        DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
                        return Commands.none();
                }
        }

        public Command getProcessorSetpointCommand() {
                Command c = new ParallelRaceGroup(
                                new SequentialCommandGroup(
                                                new InstantCommand(
                                                                () -> pivot.changeSetpoint(
                                                                                PivotConstants.kAlgaeSafeSetpoint)),
                                                new InstantCommand(
                                                                () -> elevator.ChangeSetpoint(
                                                                                Elevator.kProcessorSetpoint)),
                                                Commands.waitUntil(
                                                                () -> elevator.atSetpoint(Elevator.kProcessorSetpoint))
                                                                .andThen(() -> pivot
                                                                                .changeSetpoint(PivotConstants.kAlgaeScoringSetpoint))),
                                new RunCommand(() -> elevator.runElevatorAlgae(
                                                () -> pivot.atSetpoint(PivotConstants.kAlgaeSafeSetpoint))));
                c.addRequirements(elevator);
                return c;
        }

        public Command getBargeSetpointCommand() {
                Command c = new ParallelRaceGroup(
                                new SequentialCommandGroup(
                                                new InstantCommand(
                                                                () -> pivot.changeSetpoint(
                                                                                PivotConstants.kAlgaeSafeSetpoint)),
                                                new InstantCommand(() -> elevator.ChangeSetpoint(Elevator.kL4Setpoint)),
                                                Commands.waitUntil(() -> elevator.atSetpoint(0.0)).andThen(
                                                                () -> pivot.changeSetpoint(
                                                                                PivotConstants.kAlgaeScoringSetpoint))),
                                new RunCommand(() -> elevator.runElevatorAlgae(
                                                () -> pivot.atSetpoint(PivotConstants.kAlgaeSafeSetpoint))));
                c.addRequirements(elevator);
                return c;
        }

        public Command getL4SetpointCommand() {
                Command c = new ParallelRaceGroup(
                                new SequentialCommandGroup(
                                                new InstantCommand(
                                                                () -> pivot.changeSetpoint(
                                                                                PivotConstants.kElevatingSetpoint)),
                                                new InstantCommand(() -> elevator.ChangeSetpoint(Elevator.kL4Setpoint)),
                                                Commands.waitUntil(() -> elevator.atSetpoint(Elevator.kL4Setpoint))
                                                                .andThen(() -> pivot.changeSetpoint(
                                                                                PivotConstants.kL4Setpoint))),
                                new RunCommand(() -> elevator
                                                .runElevator(() -> pivot
                                                                .atSetpoint(PivotConstants.kElevatingSetpoint))));
                c.addRequirements(elevator);
                return c;
        }

        public Command getL3SetpointCommand() {
                Command c = new ParallelRaceGroup(
                                new SequentialCommandGroup(
                                                new InstantCommand(
                                                                () -> pivot.changeSetpoint(
                                                                                PivotConstants.kElevatingSetpoint)),
                                                new InstantCommand(() -> elevator.ChangeSetpoint(Elevator.kL3Setpoint)),
                                                Commands.waitUntil(() -> elevator.atSetpoint(Elevator.kL3Setpoint))
                                                                .andThen(() -> pivot.changeSetpoint(
                                                                                PivotConstants.kL3Setpoint))),
                                new RunCommand(() -> elevator
                                                .runElevator(() -> pivot
                                                                .atSetpoint(PivotConstants.kElevatingSetpoint))));
                c.addRequirements(elevator);
                return c;
        }

        public Command getL2SetpointCommand() {
                Command c = new ParallelRaceGroup(
                                new SequentialCommandGroup(
                                                new InstantCommand(
                                                                () -> pivot.changeSetpoint(
                                                                                PivotConstants.kElevatingSetpoint)),
                                                new InstantCommand(() -> elevator.ChangeSetpoint(Elevator.kL2Setpoint)),
                                                Commands.waitUntil(() -> elevator.atSetpoint(Elevator.kL2Setpoint))
                                                                .andThen(() -> pivot.changeSetpoint(
                                                                                PivotConstants.kL2Setpoint))),
                                new RunCommand(() -> elevator
                                                .runElevator(() -> pivot
                                                                .atSetpoint(PivotConstants.kElevatingSetpoint))));
                c.addRequirements(elevator);
                return c;
        }

        public Command getZeroSetpointCommand() {
                Command c = new ParallelRaceGroup(
                                new SequentialCommandGroup(
                                                new InstantCommand(
                                                                () -> pivot.changeSetpoint(
                                                                                PivotConstants.kElevatingSetpoint)),
                                                new InstantCommand(() -> elevator.ChangeSetpoint(0.0)),
                                                Commands.waitUntil(() -> elevator.atSetpoint(0.0)).andThen(
                                                                () -> pivot.changeSetpoint(
                                                                                PivotConstants.kIntakeSetpoint))),
                                new RunCommand(() -> elevator
                                                .runElevator(() -> pivot
                                                                .atSetpoint(PivotConstants.kElevatingSetpoint))));
                c.addRequirements(elevator);
                return c;
        }

        public Command getAlgaeSetpointCommand() {
                Command c = new ParallelRaceGroup(new SequentialCommandGroup(
                                new InstantCommand(() -> pivot.changeSetpoint(PivotConstants.kElevatingSetpoint)),
                                new InstantCommand(() -> elevator.ChangeSetpoint(Elevator.kAlgaeSetpoint)),
                                Commands.waitUntil(() -> elevator.atSetpoint(Elevator.kAlgaeSetpoint))
                                                .andThen(() -> pivot.changeSetpoint(PivotConstants.kAlgaeSetpoint))),
                                new RunCommand(() -> elevator
                                                .runElevator(() -> pivot
                                                                .atSetpoint(PivotConstants.kElevatingSetpoint))));
                c.addRequirements(elevator);
                return c;
        }

        public Command getL2AlgaeSetpointCommand() {
                Command c = new ParallelRaceGroup(
                                new SequentialCommandGroup(
                                                new InstantCommand(
                                                                () -> pivot.changeSetpoint(
                                                                                PivotConstants.kElevatingSetpoint)),
                                                new InstantCommand(() -> elevator.ChangeSetpoint(Elevator.kL2Setpoint)),
                                                Commands.waitUntil(() -> elevator.atSetpoint(Elevator.kL2Setpoint))
                                                                .andThen(
                                                                                () -> pivot.changeSetpoint(
                                                                                                PivotConstants.kAlgaeSetpoint))),
                                new RunCommand(() -> elevator
                                                .runElevator(() -> pivot
                                                                .atSetpoint(PivotConstants.kElevatingSetpoint))));
                c.addRequirements(elevator);
                return c;
        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
                // Pathfinding.setPathfinder(new LocalADStar());

                // try{
                // // Load the path we want to pathfind to and follow
                // PathPlannerPath path = PathPlannerPath.fromPathFile("New Path");
                // drivetrain.publisher1.set(path.getStartingHolonomicPose().get());
                // // // Create the constraints to use while pathfinding. The constraints
                // defined in the path will only be used for the path.
                // PathConstraints constraints = new PathConstraints(
                // 0.5, 0.5,
                // Units.degreesToRadians(180), Units.degreesToRadians(180));

                // // Since AutoBuilder is configured, we can use it to build pathfinding
                // commands
                // return AutoBuilder.pathfindThenFollowPath(
                // path,
                // constraints);
                // return AutoBuilder.followPath(path);

                // PathPlannerAuto auto = new PathPlannerAuto("Cage 4 - E (L4) - C (L4)");
                // return auto;

                // PathPlannerPath path = PathPlannerPath.fromPathFile("Angle Path");

                // RobotConfig robotConfig = RobotConfig.fromGUISettings();
                // PathPlannerTrajectory traj = path.getIdealTrajectory(robotConfig).get();

                // drivetrain.resetPose(
                // AllianceFlipUtil.apply(path.getStartingHolonomicPose().get())
                // );
                // return AutoBuilder.followPath(path);
                // } catch (Exception e) {
                // DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
                // return Commands.none();
                // }
        }

        public Command constructAligningCommand(boolean isLeftAlign) {
                Pose2d tagPose = new Pose2d();
                Integer targetId = 7;
                double xMagnitude = Constants.Drivetrain.kXShiftMagnitude;
                double yMagnitude = Constants.Drivetrain.kYShiftMagnitude;

                List<Integer> targetTagSet;
                Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                        targetTagSet = alliance.get() == DriverStation.Alliance.Red ? Arrays.asList(7, 8, 9, 10, 11, 6)
                                        : Arrays.asList(21, 20, 19, 18, 17, 22);
                } else {
                        return null;
                }

                double minDistance = Double.MAX_VALUE;
                for (int tag : targetTagSet) {
                        Pose2d pose = photonVision.at_field.getTagPose(tag).orElse(new Pose3d()).toPose2d();
                        Translation2d translate = pose.minus(drivetrain.getPose()).getTranslation();
                        double distance = translate.getNorm();

                        if (distance < minDistance) {
                                tagPose = pose;
                                targetId = tag;
                                minDistance = distance;
                        }
                }

                double angle = Units.degreesToRadians(60 * targetTagSet.indexOf(targetId));
                double offset = Units.degreesToRadians(isLeftAlign ? 90 : -90);

                double x = xMagnitude * Math.cos(angle) + yMagnitude * Math.cos(angle + offset);
                double y = xMagnitude * Math.sin(angle) + yMagnitude * Math.sin(angle + offset);

                PathConstraints constraints = new PathConstraints(3.0, 4.0, Units.degreesToRadians(540),
                                Units.degreesToRadians(720));
                return AutoBuilder.pathfindToPose(
                                new Pose2d(tagPose.getX() + x, tagPose.getY() + y,
                                                tagPose.getRotation().plus(Rotation2d.fromRadians(Math.PI / 2.0))),
                                constraints);
        }

        public void robotPeriodic() {
                photonPoseUpdate();
                SmartDashboard.putNumber("Battery Voltage", powerDistribution.getVoltage());
                SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
                autoField.setRobotPose(drivetrain.getPose());
        }

        public void autonomousInit() {
                Elastic.selectTab("Autonomous");
                leds.set(LEDs.kParty_Palette_Twinkles);
        }

        public void autonomousPeriodic() {

        }

        public void teleopInit() {
                leds.setAllianceColor();
                Elastic.selectTab("Teleoperated");
        }

        public void teleopPeriodic() {
                // try {
                // PathPlannerPath paths = PathPlannerPath.fromPathFile("New Path");
                // drivetrain.publisher1.set(AllianceFlipUtil.apply(paths.getStartingHolonomicPose().get()));
                // } catch (Exception e) {
                // DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
                // }
                // SmartDashboard.putNumber("Raw X Speed",
                // -MathUtil.applyDeadband(Driver1.getRawAxis(1),
                // Operator.kDriveDeadband));
                // SmartDashboard.putNumber("Raw Y Speed",
                // -MathUtil.applyDeadband(Driver1.getRawAxis(0),
                // Operator.kDriveDeadband));
        }

        public void disabledPeriodic() {
                newAutoName = getAutonomousCommand().getName();
                alliance = DriverStation.getAlliance();
                if (autoName != newAutoName || alliance != lastAlliance) {
                        autoName = newAutoName;
                        lastAlliance = alliance;
                        if (AutoBuilder.getAllAutoNames().contains(autoName)) {
                                try {
                                        List<PathPlannerPath> pathPlannerPaths = PathPlannerAuto
                                                        .getPathGroupFromAutoFile(autoName);
                                        List<Pose2d> poses = new ArrayList<>();
                                        for (PathPlannerPath path : pathPlannerPaths) {

                                                if (DriverStation.getAlliance().equals(Optional.of(Alliance.Red))) {
                                                        poses.addAll(path.getAllPathPoints().stream()
                                                                        .map(point -> new Pose2d(
                                                                                        Field.fieldLength
                                                                                                        - point.position.getX(),
                                                                                        Field.fieldWidth - point.position
                                                                                                        .getY(),
                                                                                        new Rotation2d()))
                                                                        .collect(Collectors.toList()));
                                                } else {
                                                        poses.addAll(path.getAllPathPoints().stream()
                                                                        .map(point -> new Pose2d(point.position.getX(),
                                                                                        point.position.getY(),
                                                                                        new Rotation2d()))
                                                                        .collect(Collectors.toList()));
                                                }
                                        }
                                        autoField.getObject("path").setPoses(poses);
                                } catch (IOException e) {
                                        e.printStackTrace();
                                        return;
                                } catch (ParseException e) {
                                        e.printStackTrace();
                                        return;
                                }
                        }
                }
        }

        public static void photonPoseUpdate() {
                Optional<EstimatedRobotPose> photonPoseOptional = photonVision.getCam1Pose();

                if (photonPoseOptional.isPresent()) {
                        Pose3d photonPose = photonPoseOptional.get().estimatedPose;

                        if (photonPose.getX() >= 0 && photonPose.getX() <= Field.fieldLength
                                        && photonPose.getY() >= 0 && photonPose.getY() <= Field.fieldWidth
                                        && photonVision.getCam1BestTarget() != null) {

                                Pose2d closestTag = photonVision.at_field
                                                .getTagPose(photonVision.getCam1BestTarget().getFiducialId()).get()
                                                .toPose2d();
                                Translation2d translate = closestTag.minus(photonPose.toPose2d()).getTranslation();

                                double distance = translate.getNorm();
                                double xStddev = Math.pow(distance, 2) / 8.0088;
                                double yStddev = xStddev;
                                double rotStddev = Units.degreesToRadians(120.0);
                                drivetrain.publisher3.set(photonPose.toPose2d());
                                drivetrain.m_poseEstimator
                                                .setVisionMeasurementStdDevs(
                                                                VecBuilder.fill(xStddev, yStddev, rotStddev));
                                drivetrain.addVisionMeasurement(photonPose.toPose2d(),
                                                photonPoseOptional.get().timestampSeconds);
                                drivetrain.publisher3.set(photonPose.toPose2d());
                        }
                }

                // TODO: Fix this commented out portion

                photonPoseOptional = photonVision.getCam2Pose();

                if (photonPoseOptional.isPresent()) {
                        Pose3d photonPose = photonPoseOptional.get().estimatedPose;

                        if (photonPose.getX() >= 0 && photonPose.getX() <= Field.fieldLength
                                        && photonPose.getY() >= 0 && photonPose.getY() <= Field.fieldWidth
                                        && photonVision.getCam2BestTarget() != null) {

                                Pose2d closestTag = photonVision.at_field
                                                .getTagPose(photonVision.getCam2BestTarget().getFiducialId()).get()
                                                .toPose2d();
                                Translation2d translate = closestTag.minus(photonPose.toPose2d()).getTranslation();

                                double distance = translate.getNorm();
                                double xStddev = Math.pow(distance, 2)/ 8.0088;
                                double yStddev = xStddev;
                                double rotStddev = Units.degreesToRadians(120.0);
                                drivetrain.publisher4.set(photonPose.toPose2d());
                                drivetrain.m_poseEstimator
                                                .setVisionMeasurementStdDevs(
                                                                VecBuilder.fill(xStddev, yStddev, rotStddev));
                                drivetrain.addVisionMeasurement(photonPose.toPose2d(),
                                                photonPoseOptional.get().timestampSeconds);

                                drivetrain.publisher4.set(photonPose.toPose2d());
                        }
                }
        }
}
