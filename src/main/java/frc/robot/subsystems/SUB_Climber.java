package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Climber;



public class SUB_Climber extends SubsystemBase {
    private static SUB_Climber INSTANCE = null;
    private SparkMax climber = new SparkMax(Climber.kClimberCanID, MotorType.kBrushless);
    private SparkMaxConfig climberConfig = new SparkMaxConfig();

    public SUB_Climber() {
        climberConfig.inverted(false);
        climberConfig.voltageCompensation(12);
        climberConfig.smartCurrentLimit(60);
        climber.configure(climberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void move(double speed) {
        climber.set(speed);
    }

    public static SUB_Climber getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new SUB_Climber();
        }
        return INSTANCE;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}