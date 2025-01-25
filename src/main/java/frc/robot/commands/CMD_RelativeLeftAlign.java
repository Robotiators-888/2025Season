// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.SUB_Drivetrain;
import frc.robot.subsystems.SUB_PhotonVision;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CMD_RelativeLeftAlign extends Command {
  SUB_PhotonVision photonVision;
  SUB_Drivetrain drivetrain;

  Pose2d goalPose;
  double goalAngle;

  CommandXboxController driverController;

  private final PIDController xController = new PIDController(0.25, 0, 0); // 3, 0, 0
  private final PIDController yController = new PIDController(0.25, 0, 0);
  private final PIDController robotAngleController = new PIDController(0.5, 0.01, 0); // 0.25, 0, 0

  /** Creates a new CMD_AdjustPivotOnDist. */
  public CMD_RelativeLeftAlign(SUB_Drivetrain drivetrain, SUB_PhotonVision photonVision) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.photonVision = photonVision;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    

  }
  //25 x 30

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {



  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
