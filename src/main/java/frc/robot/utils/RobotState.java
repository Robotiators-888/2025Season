
package frc.robot.utils;

import frc.robot.Constants.Elevator;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.*;

public class RobotState {
    
    public static SUB_Elevator elevator = SUB_Elevator.getInstance();
    public static SUB_Roller roller = SUB_Roller.getInstance();
    public static SUB_Pivot pivot = SUB_Pivot.getInstance(roller.getAbsoluteEncoder());


    public static double getCorrectPivotSetpoint() {
        double currentPosition = elevator.getCurrentPosition();

        boolean isMovingUp = elevator.goal.position > currentPosition;
        boolean isMovingDown = elevator.goal.position < currentPosition;

        double lowerBoundCollisionRange = 0;

        if (currentPosition >= Elevator.kL4Setpoint) {
            return 0.;
        }

        if (currentPosition >= Elevator.kL3Setpoint){
            return 0.;
        }

        if (currentPosition >= Elevator.kL2Setpoint) {
            return 0.;
        }

        return PivotConstants.kElevatingSetpoint;

    }


}