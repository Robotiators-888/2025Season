
package frc.robot.utils;

import java.util.function.Supplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.Elevator;
import frc.robot.subsystems.*;
import frc.robot.Constants.CollisionAvoidance;

public class ElevatorManager {

    public static SUB_Elevator elevator = SUB_Elevator.getInstance();
    public static SUB_Roller roller = SUB_Roller.getInstance();
    public static SUB_Pivot pivot = SUB_Pivot.getInstance(roller.getAbsoluteEncoder());


    public double getSafeElevatingSetpoint(ScoringSetpoint startingSetpoint, ScoringSetpoint endingSetpoint){
        
        if (endingSetpoint.equals(CollisionAvoidance.bargeSetpoint) || endingSetpoint.equals(CollisionAvoidance.l1AlgaeSetpoint) || endingSetpoint.equals(CollisionAvoidance.l2AlgaeSetpoint)) {

        }
    }

}