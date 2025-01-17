
package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class SUB_Shooter extends SubsystemBase {
  public static SUB_Shooter INSTANCE = null;

  public InterpolatingDoubleTreeMap distToTimeMap = new InterpolatingDoubleTreeMap();
  private SparkPIDController PIDController; 
  public static int MANUAL_RPM = 1000;
  public static int AUTO_RPM = 1000;

  public static int SetpointRPM;
  SparkMax shooterLeft;
  SparkMax shooterRight;

  public static SUB_Shooter getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new SUB_Shooter();
    }

    return INSTANCE;
  }

  private SUB_Shooter() {
    shooterLeft = new SparkMax(30, MotorType.kBrushless);
    shooterRight = new SparkMax(31, MotorType.kBrushless);
    PIDController = shooterLeft.getPIDController();
    shooterLeft.restoreFactoryDefaults();
    shooterRight.restoreFactoryDefaults();
    //shooterRight.setInverted(false);
    shooterRight.follow(shooterLeft, false);
    PIDController.setOutputRange(-1, 1);
    shooterLeft.getEncoder().setVelocityConversionFactor(1);
    shooterLeft.enableVoltageCompensation(12);
    setPIDF(PIDController, 0, 0, 0, 1.0 / 5800.0 * (3000.0 / 2600.0));
    Timer.delay(.1);

    shooterLeft.burnFlash();
    shooterRight.burnFlash();

    SetpointRPM = 1000;

  }

  public void setMotorSpeed(double speed) {
    shooterLeft.set(speed);
  }

  public void periodic() {
    SmartDashboard.putNumber("Shooter/Shooter RPM", shooterLeft.getEncoder().getVelocity());
  }

  public void setPIDF(SparkPIDController pid, double P, double I, double D, double F) {
    pid.setP(P);
    pid.setI(I);
    pid.setD(D);
    pid.setFF(F);
  }

  public double getFlywheelRPM() {
    SmartDashboard.putNumber("FF", PIDController.getFF());
    ;
    return shooterRight.getEncoder().getVelocity();
  }

  public void shootFlywheelOnRPM(double rpm) {
    PIDController.setReference(rpm, ControlType.kVelocity);
  }

  public void setRPM(int rpm) {
    SetpointRPM = rpm;
  }

  public void setAutoRPM(int rpm) {
    AUTO_RPM = rpm;
  }

}