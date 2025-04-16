package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.GroundPivot;

public class SUB_GroundPivot extends SubsystemBase{
    private static SUB_GroundPivot INSTANCE = null;

    // I could use public SUB_GroundPivot () {} to set configs

    private SparkMax groundPivot = new SparkMax(GroundPivot.kGroundPivotCanID, MotorType.kBrushless);
    private SparkMaxConfig config = new SparkMaxConfig();
    private SparkAbsoluteEncoder absoluteEncoder = groundPivot.getAbsoluteEncoder();
    private RelativeEncoder relativeEncoder = groundPivot.getEncoder();
    private PIDController voltagePID = new PIDController(4, 0, 0.8); // TODO: Change
    public double activesetpoint = Constants.GroundPivot.kStowPos;

    public SUB_GroundPivot() {
      config.smartCurrentLimit(25);
      config.inverted(false);
      config.voltageCompensation(12);
      voltagePID.setTolerance(5);
      groundPivot.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    // Allows for changing the pivot angle of the ground intake and is needed to be able to pick up coral and score also may not be needed
    public void setGroundPivot (double percent) {
        groundPivot.set(percent);
    }

    public void runGroundPivotManualVoltage (double volts) {
      groundPivot.setVoltage(volts);
    }

    public double getGroundPivotAbsoluteEncodeValue() {
      return absoluteEncoder.getPosition();
    }

    public void changeSetpoint (double setpoint) {
      activesetpoint = setpoint;
    }

    public void drivePivotPID() {
      double outputVoltage = MathUtil.clamp(voltagePID.calculate(relativeEncoder.getPosition(), activesetpoint), -6, 6);
      runGroundPivotManualVoltage(outputVoltage);
    }

    public void zeroEncoder(){
      relativeEncoder.setPosition(0);
    }

    public boolean nearIntakeSetpoint() {
      return relativeEncoder.getPosition() > Constants.GroundPivot.kIntakePos - Constants.GroundPivot.kIntakeThreshold;
    }

    public static SUB_GroundPivot getInstance() {
        if (INSTANCE == null) {
          INSTANCE = new SUB_GroundPivot();
        }
        return INSTANCE;
      }

      public void periodic() {
        SmartDashboard.putNumber("Ground Pivot Relative Encoder", relativeEncoder.getPosition());
        SmartDashboard.putNumber("OutputVoltage", MathUtil.clamp(voltagePID.calculate(relativeEncoder.getPosition(), activesetpoint), -6, 6));
      }

}
