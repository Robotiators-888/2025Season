package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GroundPivot;

public class SUB_GroundPivot extends SubsystemBase{
    private static SUB_GroundPivot INSTANCE = null;

    // I could use public SUB_GroundPivot () {} to set configs

    private SparkMax groundPivot = new SparkMax(GroundPivot.kGroundPivotCanID, MotorType.kBrushless);


    // Allows for changing the pivot angle of the ground intake and is needed to be able to pick up coral and score
    public void setGroundPivot (double percent) {
        groundPivot.set(percent);
    }

    public static SUB_GroundPivot getInstance() {
        if (INSTANCE == null) {
          INSTANCE = new SUB_GroundPivot();
        }
        return INSTANCE;
      }

}
