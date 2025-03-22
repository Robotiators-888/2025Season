package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GroundIntake;

public class SUB_GroundIntake extends SubsystemBase{
    private static SUB_GroundIntake INSTANCE = null;

    // I could use public SUB_GroundIntake () {} to set configs

    private SparkMax groundIntake = new SparkMax(GroundIntake.kGroundIntakeCanID, MotorType.kBrushless);
    private SparkMax groundIntakePivot = new SparkMax(GroundIntake.kGroundIntakePivotCanID, MotorType.kBrushless);

    // Allows for running the ground intake and can be used to intake and score the coral by using negative values
    public void setGroundIntakeOutput (double percent) {
        groundIntake.set(percent);
    }

    // Allows for changing the pivot angle of the ground intake and is needed to be able to pick up coral and score
    public void setGroundIntakePivotOutput (double percent) {
        groundIntakePivot.set(percent);
    }

    public static SUB_GroundIntake getInstance() {
        if (INSTANCE == null) {
          INSTANCE = new SUB_GroundIntake();
        }
        return INSTANCE;
      }

}
