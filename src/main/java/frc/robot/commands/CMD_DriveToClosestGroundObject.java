package frc.robot.commands;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PhotonVision;
import frc.robot.subsystems.SUB_Drivetrain;
import frc.robot.subsystems.SUB_PhotonVision;

public class CMD_DriveToClosestGroundObject extends Command {
  private final SUB_Drivetrain drivetrain;
  private final SUB_PhotonVision photonVision;
  private final PhotonCamera cam3;

  private final PIDController xController = new PIDController(1.0, 0, 0);
  private final PIDController yController = new PIDController(1.0, 0, 0);
  private final PIDController rotController = new PIDController(1.0, 0, 0);

  private PhotonTrackedTarget closestCoral;
  private double lastCoralArea = 0;
  private double lastCoralYaw = 0;
  private double lastCoralPitch = 0;
  private boolean CoralFound = false;

  public CMD_DriveToClosestGroundObject(SUB_Drivetrain drivetrain, SUB_PhotonVision photonVision) {
    this.drivetrain = drivetrain;
    this.photonVision = photonVision;
    this.cam3 = new PhotonCamera(PhotonVision.kCam3Name);
    cam3.setPipelineIndex(2); // HSV pipeline index

    rotController.enableContinuousInput(-Math.PI, Math.PI);
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    xController.reset();
    yController.reset();
    rotController.reset();

    cam3.setPipelineIndex(2);
    CoralFound = false;
    closestCoral = null;
  }

  @Override
  public void execute() {
    List<PhotonPipelineResult> results = cam3.getAllUnreadResults();
    PhotonPipelineResult result = results.isEmpty() ? new PhotonPipelineResult() : results.get(results.size() - 1);

    if (result.hasTargets()) {
      CoralFound = true;
      closestCoral = result.getBestTarget();

      if (closestCoral != null) {
        lastCoralArea = closestCoral.getArea();
        lastCoralYaw = closestCoral.getYaw();
        lastCoralPitch = closestCoral.getPitch();

        double yawRadians = Math.toRadians(lastCoralYaw);
        double rotSpeed = rotController.calculate(0, yawRadians);
        double forwardSpeed = 1.0 - (lastCoralArea / 100.0);
        forwardSpeed = Math.max(0.2, Math.min(forwardSpeed, 0.8));
        double lateralSpeed = -yController.calculate(0, yawRadians);

        ChassisSpeeds speeds = new ChassisSpeeds(forwardSpeed, lateralSpeed, rotSpeed);
        drivetrain.driveFieldRelative(speeds);

        SmartDashboard.putNumber("HSV Target Yaw", lastCoralYaw);
        SmartDashboard.putNumber("HSV Target Area", lastCoralArea);
        SmartDashboard.putNumber("HSV Forward Speed", forwardSpeed);
        SmartDashboard.putNumber("HSV Lateral Speed", lateralSpeed);
        SmartDashboard.putNumber("HSV Rotation Speed", rotSpeed);
      }
    } else {
      CoralFound = false;
      drivetrain.stop();
    }
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
    cam3.setPipelineIndex(0);
  }

  @Override
  public boolean isFinished() {
    return CoralFound && lastCoralArea > 80;
  }
}
