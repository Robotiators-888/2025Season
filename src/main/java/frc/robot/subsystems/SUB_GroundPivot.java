package frc.robot.subsystems;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GroundPivot;

public class SUB_GroundPivot extends SubsystemBase{
    private static SUB_GroundPivot INSTANCE = null;

    // I could use public SUB_GroundPivot () {} to set configs

    private SparkMax groundPivot = new SparkMax(GroundPivot.kGroundPivotCanID, MotorType.kBrushless);
    private SparkAbsoluteEncoder groundPivotAbsoluteEncoder = groundPivot.getAbsoluteEncoder();
    private PIDController voltagePID = new PIDController(0.035, 0, 0.0035); // TODO: Change
    public double pivotAngleGhost = 0.0;

    public SUB_GroundPivot() {
      
    }

    // Allows for changing the pivot angle of the ground intake and is needed to be able to pick up coral and score also may not be needed
    public void setGroundPivot (double percent) {
        groundPivot.set(percent);
    }

    public void runGroundPivotManualVoltage (double volts) {
      groundPivot.setVoltage(volts);
    }

    public double getGroundPivotAbsoluteEncodeValue() {
      return groundPivotAbsoluteEncoder.getPosition();
    }

    public void changeGroundPivotAngleGhost (double ghost) {
      pivotAngleGhost = ghost;
    }

    public void setGroundPivotAngleToGhost() {
      double outputVoltage = MathUtil.clamp(voltagePID.calculate(getGroundPivotAbsoluteEncodeValue(), pivotAngleGhost), -3, 3);
      runGroundPivotManualVoltage(outputVoltage);
    }

    public static SUB_GroundPivot getInstance() {
        if (INSTANCE == null) {
          INSTANCE = new SUB_GroundPivot();
        }
        return INSTANCE;
      }

}
