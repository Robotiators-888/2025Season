
package frc.robot.subsystems;


import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class SUB_Intake extends SubsystemBase {

  public static SUB_Intake INSTANCE;
  SparkMax intakeMotor;
  Boolean intakeBool;
  boolean hasNote;
  SparkMaxConfig config = new SparkMaxConfig();

  /** Creates a new SUB_Intake. */
  private SUB_Intake() {

    intakeMotor = new SparkMax(Constants.Intake.kINTAKE_MOTOR_CANID, MotorType.kBrushless);
    config.voltageCompensation(60);
    config.idleMode(IdleMode.kCoast);
    config.inverted(true);
    intakeMotor.configure(config,
        ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
 
  }

  public static SUB_Intake getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new SUB_Intake();
    }

    return INSTANCE;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Intake/RPM", intakeMotor.getEncoder().getVelocity());
    // SmartDashboard.putNumber(getName(), 0)
    SmartDashboard.putNumber("Intake/Speed (m/sec)",
        (((intakeMotor.getEncoder().getVelocity() * 2 * Math.PI) / 8)) / 60);

  }

  /**
   * Set Speed of intake
   * 
   * @param speed Percent speed of motor
   */
  public void setMotorSpeed(double speed) {
    intakeMotor.set(speed);
  }

  public boolean intakeHasNote() {
    return hasNote;
  }

  public void setHasNote(boolean n) {
    hasNote = n;
  }
}
