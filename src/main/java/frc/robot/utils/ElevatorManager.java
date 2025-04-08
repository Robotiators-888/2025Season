
package frc.robot.utils;

import java.text.CollationElementIterator;
import java.util.function.Supplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.Elevator;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.*;
import frc.robot.Constants.CollisionAvoidance;

public class ElevatorManager {

    public static SUB_Elevator elevator = SUB_Elevator.getInstance();
    public static SUB_Roller roller = SUB_Roller.getInstance();
    public static SUB_Pivot pivot = SUB_Pivot.getInstance(roller.getAbsoluteEncoder());


    public static Double getSafeElevatingSetpoint(ScoringSetpoint startingSetpoint, ScoringSetpoint endingSetpoint){

        if (startingSetpoint == null || endingSetpoint == null){
            return PivotConstants.kElevatingSetpoint;
        }
        
        boolean endingIsAlgae = endingSetpoint.equals(CollisionAvoidance.bargeSetpoint) || endingSetpoint.equals(CollisionAvoidance.l1AlgaeSetpoint) || endingSetpoint.equals(CollisionAvoidance.l2AlgaeSetpoint) || endingSetpoint.equals(CollisionAvoidance.processorSetpoint);
        if (startingSetpoint.equals(CollisionAvoidance.l4Setpoint)){

            // DO NOT ALLOW ELEVATOR TO MOVE TO ANY ALGAE SETPOINT
            if (endingIsAlgae) {
                return null;
            }

            return PivotConstants.kElevatingSetpoint;
        }

        if (startingSetpoint.equals(CollisionAvoidance.bargeSetpoint)) {
            
            // Physically not a possible move
            if (endingSetpoint.equals(CollisionAvoidance.l4Setpoint)){
                return null;
            }

            return PivotConstants.kAlgaeSafeSetpoint;
        }

        if (endingIsAlgae){
            return PivotConstants.kAlgaeSafeSetpoint;
        }
        return PivotConstants.kElevatingSetpoint;
    }

      public static ScoringSetpoint getCurrentSetpoint() {
        double pivotReading =  pivot.getCurrentPosition();
        double elevatorReading = elevator.getCurrentPosition();

        for (ScoringSetpoint s: new ScoringSetpoint[]{CollisionAvoidance.bargeSetpoint, CollisionAvoidance.l1AlgaeSetpoint, CollisionAvoidance.l2AlgaeSetpoint, CollisionAvoidance.l1Setpoint, CollisionAvoidance.l2Setpoint, CollisionAvoidance.l3Setpoint, CollisionAvoidance.l4Setpoint}){
            
            if (s.atSetpoint(pivotReading, elevatorReading)){
                return s;
            }
        }
        return null;
      }

}