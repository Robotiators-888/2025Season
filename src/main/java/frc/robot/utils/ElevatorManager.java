
package frc.robot.utils;

import java.util.function.Supplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.Elevator;
import frc.robot.subsystems.*;

public class ElevatorManager {

    public static SUB_Elevator elevator = SUB_Elevator.getInstance();
    public static SUB_Roller roller = SUB_Roller.getInstance();
    public static SUB_Pivot pivot = SUB_Pivot.getInstance(roller.getAbsoluteEncoder());

    public void runElevator(Supplier<Boolean> pivotSafe) {

        double activesetpoint = elevator.getActiveSetpoint();
        RelativeEncoder primaryencoder = elevator.getPrimaryEncoder();
        SparkMax primary = elevator.getPrimary();

        if (elevator.getCurrentPosition() >= Elevator.kResetHomingThreshold) {
            SmartDashboard.putBoolean("EMERGENCY HOMED!!!", false);
            SmartDashboard.putBoolean("Homed", false);
        }
        SmartDashboard.putNumber("Elevator Output Voltage", primary.getAppliedOutput() * primary.getBusVoltage());
        SmartDashboard.putNumber("EncoderPos", primaryencoder.getPosition());
        if (!pivotSafe.get() && !elevator.atSetpoint()) {
            SmartDashboard.putBoolean("Elevator is Safe", false);
            if (roller.getHasCoral()) {
                elevator.runElevatorManualVoltage(Elevator.kCoralHoldingVoltage);
                return;
            }
            if (roller.getHasAlgae()) {
                elevator.runElevatorManualVoltage(Elevator.kAlgaeHoldingVoltage);
                return;
            }
            if (primaryencoder.getPosition() > .6) {
                elevator.runElevatorManualVoltage(Elevator.kEmptyHoldingVoltageTop);
                return;
            }
            elevator.runElevatorManualVoltage(Elevator.kEmptyHoldingVoltage);
            return;
        }

        SmartDashboard.putBoolean("Elevator is Safe", false);
        if (activesetpoint <= 0 && elevator.getCurrentPosition() <= Elevator.kSlowDownThreshold) {
            elevator.HomeElevator();
            return;
        }

        if (Math.abs(activesetpoint - primaryencoder.getPosition()) < .02) {
            if (roller.getHasCoral()) {
                elevator.runElevatorManualVoltage(Elevator.kCoralHoldingVoltage);
                return;
            }
            if (roller.getHasAlgae()) {
                elevator.runElevatorManualVoltage(Elevator.kAlgaeHoldingVoltage);
                return;
            }
            if (primaryencoder.getPosition() > .6) {
                elevator.runElevatorManualVoltage(Elevator.kEmptyHoldingVoltageTop);
                return;
            }
            elevator.runElevatorManualVoltage(Elevator.kEmptyHoldingVoltage);
            return;
        }
        if (activesetpoint > elevator.getCurrentPosition()) {
            if (activesetpoint - elevator.getCurrentPosition() > Elevator.kMaxUpErrorThreshold) {
                elevator.runElevatorManualVoltage(Elevator.kMaxUpVoltage);
                return;
            }
            if (activesetpoint - elevator.getCurrentPosition() > Elevator.kHighUpErrorThreshold) {
                elevator.runElevatorManualVoltage(Elevator.kHighUpVoltage);
                return;
            }
            if (activesetpoint - elevator.getCurrentPosition() > Elevator.kMediumUpErrorThreshold) {
                elevator.runElevatorManualVoltage(Elevator.kMediumUpVoltage);
                return;
            }
            elevator.runElevatorManualVoltage(Elevator.kSlowUpVoltage);
            return;
        }
        if (activesetpoint < elevator.getCurrentPosition()) {
            if (Math.abs(elevator.getCurrentPosition() - activesetpoint) > Elevator.kMaxDownErrorThreshold) {
                elevator.runElevatorManualVoltage(Elevator.kMaxDownVoltage);
                return;
            }
            if (Math.abs(elevator.getCurrentPosition() - activesetpoint) > Elevator.kHighDownErrorThreshold) {
                elevator.runElevatorManualVoltage(Elevator.kHighDownVoltage);
                return;
            }
            if (Math.abs(elevator.getCurrentPosition() - activesetpoint) > Elevator.kMediumDownErrorThreshold) {
                elevator.runElevatorManualVoltage(Elevator.kMediumDownVoltage);
                return;
            }
            elevator.runElevatorManualVoltage(Elevator.kSlowDownThreshold);
            return;
        }
        elevator.runElevatorManualVoltage(0);
    }

}