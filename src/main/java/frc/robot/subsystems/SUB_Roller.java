// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import java.util.function.BooleanSupplier;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Roller;


public class SUB_Roller extends SubsystemBase {
  private static SUB_Roller INSTANCE = null;
  private SparkMax roller = new SparkMax(Roller.kRollerCanID, MotorType.kBrushless);
  private SparkMaxConfig config = new SparkMaxConfig();
  private RelativeEncoder encoder = roller.getEncoder();
  private SparkAbsoluteEncoder absoluteEncoder = roller.getAbsoluteEncoder();
  private Boolean hasCoral = false;
  private Timer timer = new Timer();

  private SUB_Roller() {
    config.voltageCompensation(12);
    config.inverted(false);
    config.smartCurrentLimit(Roller.kRollerCurrentLimit);
    config.absoluteEncoder.positionConversionFactor(360);
    config.absoluteEncoder.velocityConversionFactor(360 / 60);

    roller.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  public boolean atCurrentThresholdandTimerElapsed() {
    return roller.getOutputCurrent() > Roller.kIntakeCurrentThreshold
        && timer.get() > Roller.kIntakeStartingTime;
  }

  public BooleanSupplier isFreeSpinning() {
    return () -> encoder.getVelocity() >= Roller.kFreeSpinThreshold;
  }

  public void timerInteract(boolean start) {
    if (start) {
      timer.reset();
      timer.start();
    } else {
      timer.stop();
      timer.reset();
    }

  }

  public void setRollerOutput(double percent) {
    roller.set(percent);
  }

  public boolean hasCoral() {
    return hasCoral;
  }

  public void hasCoral(boolean hasCoral) {
    this.hasCoral = hasCoral;
  }

  public SparkAbsoluteEncoder getAbsoluteEncoder() {
    return absoluteEncoder;
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
