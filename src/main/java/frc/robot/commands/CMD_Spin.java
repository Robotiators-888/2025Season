package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SUB_Drivetrain;

public class CMD_Spin extends Command {
    private SUB_Drivetrain drivetrain;
    public CMD_Spin(SUB_Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        drivetrain.drive(0, 0, -1, true, true);

    }
}
