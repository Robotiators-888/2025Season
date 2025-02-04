
package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SUB_Shooter extends SubsystemBase {
  public static SUB_Shooter INSTANCE = null;

  public InterpolatingDoubleTreeMap distToTimeMap = new InterpolatingDoubleTreeMap();
  public static int MANUAL_RPM = 1000;
  public static int AUTO_RPM = 1000;
  private SparkClosedLoopController PIDController;
  public static int SetpointRPM;
  SparkMax ShooterLeft = new SparkMax(30, MotorType.kBrushless);
  SparkMax ShooterRight = new SparkMax(31, MotorType.kBrushless);

  public static final SparkMaxConfig shooterConfig = new SparkMaxConfig();

  public void setShooterLR(SparkMax shooterLeft, SparkMax shooterRight) {
    ShooterLeft = new SparkMax(30, MotorType.kBrushless);
    ShooterRight = new SparkMax(31, MotorType.kBrushless);

    config.voltageCompensation(60);
    config.idleMode(IdleMode.kCoast);
    ShooterLeft.configure(shooterConfig,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    ShooterRight.configure(shooterConfig,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
 
  }

  public static SUB_Shooter getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new SUB_Shooter();
    }

    return INSTANCE;
  }

  private SUB_Shooter() {

    PIDController = ShooterLeft.getClosedLoopController();
    PIDController = ShooterRight.getClosedLoopController();

    config
        .inverted(true)
        .idleMode(IdleMode.kBrake);
    config.encoder
        .positionConversionFactor(1000)
        .velocityConversionFactor(1000);
    config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(1.0, 0.0, 0.0);

    SetpointRPM = 1000;
  }

  public double getFlywheelRPM() {
    return ShooterRight.getEncoder().getVelocity();
  }

  public void shootFlywheelOnRPM(double rpm) {
    PIDController.setReference(rpm, ControlType.kVelocity);
  }

  public void setMotorSpeed(double speed) {
    ShooterLeft.set(speed);
  }

  public void periodic() {
    SmartDashboard.putNumber("Shooter/Shooter RPM", ShooterLeft.getEncoder().getVelocity());
  }

  SparkMaxConfig config = new SparkMaxConfig();

}