// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;



import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SUB_Index extends SubsystemBase {

  private static SUB_Index INSTANCE = null;
  SparkMax indexLeft;
  SparkMax indexRight;
  SparkMaxConfig config = new SparkMaxConfig();
  DigitalInput dio9 = new DigitalInput(9);
  Timer currentTimer = new Timer();

  public boolean getTopBannerSensor() {
    return dio9.get();
  }

  public void starttimer() {
    currentTimer.reset();
    currentTimer.start();
  }

  public boolean CurrentLimitSpike() {
    double avg = (indexLeft.getOutputCurrent());

    SmartDashboard.putNumber("Index/OutputCurrent", avg);
    return (currentTimer.hasElapsed(.35) && avg > 15.0);
  }

  public static SUB_Index getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new SUB_Index();
    }

    return INSTANCE;
  }

  /** Creates a new SUB_Index. */
  private SUB_Index() {
    this.indexLeft = new SparkMax(32, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless); 
    this.indexRight = new SparkMax(33, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);

    config.smartCurrentLimit(40);
    config.voltageCompensation(12);
    config.idleMode(IdleMode.kBrake);
    config.inverted(true);
    indexLeft.configure(config,
        ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    config.follow(indexLeft);
    indexRight.configure(config,
        ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("Index RPM", indexLeft.getEncoder().getVelocity());
    SmartDashboard.putBoolean("Index/Banner", getTopBannerSensor());
    SmartDashboard.putNumber("Index/LeftIndexCurrent", indexLeft.getOutputCurrent());
    SmartDashboard.putNumber("Index/RightIndexCurrent", indexRight.getOutputCurrent());
    SmartDashboard.putNumber("Index/LeftIndexVelocity", indexLeft.getEncoder().getVelocity());
    SmartDashboard.putNumber("Index/RightIndexVelocity", indexRight.getEncoder().getVelocity());
  }

  public void setMotorSpeed(double speed) {

    indexLeft.set(speed);
  }
}
