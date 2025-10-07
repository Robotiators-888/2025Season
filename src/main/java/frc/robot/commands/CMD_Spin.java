package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SUB_Drivetrain;

public class CMD_Spin extends Command {
    private SUB_Drivetrain drivetrain;
    public CMD_Spin (SUB_Drivetrain drivetrain) {
        drivetrain = this.drivetrain;
        addRequirements(drivetrain);
    }
    public void execute () {
        drivetrain.drive(0,0,360,true,true);
    }
}
