// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import java.util.function.BooleanSupplier;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.networktables.BooleanArrayEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Roller;


public class SUB_Roller extends SubsystemBase {
  private static SUB_Roller INSTANCE = null;
  private static SparkMax roller = new SparkMax(Roller.kRollerCanID, MotorType.kBrushless);
  private static SparkMaxConfig config = new SparkMaxConfig();
  private static RelativeEncoder encoder = roller.getEncoder();
  private static Boolean HasCoral = false;



  private SUB_Roller() {
    config.voltageCompensation(12);
    config.inverted(false);
    config.smartCurrentLimit(40);

    roller.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public BooleanSupplier outputCurrentThreshold() {
    return () -> roller.getOutputCurrent() > Roller.kIntakeCurrentThreshold;
  }

  public void setOutput(double percent) {
    roller.set(percent);
  }

  public boolean hasCoral(){
    return HasCoral;
  }

  public void hasCoral(boolean hasCoral){
    hasCoral = !hasCoral;
  }

  public static SUB_Roller getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new SUB_Roller();
    }
    return INSTANCE;
  }

  public void periodic() {
    SmartDashboard.putNumber("Roller RPM", (encoder.getVelocity() / 60));
  }
}
