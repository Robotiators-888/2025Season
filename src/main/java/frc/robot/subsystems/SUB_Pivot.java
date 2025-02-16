// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;


public class SUB_Pivot extends SubsystemBase {
  private static SUB_Pivot INSTANCE = null;
  private SparkMax primary = new SparkMax(35, MotorType.kBrushless);
  private SparkMax secondary = new SparkMax(36, MotorType.kBrushless);
  private SparkMaxConfig config = new SparkMaxConfig();

  private TrapezoidProfile profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(0., 0.));
  private SparkAbsoluteEncoder rotateEncoder;

  private TrapezoidProfile.State goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();
  private TrapezoidProfile.State currentState = new TrapezoidProfile.State();

  private ProfiledPIDController voltagePID =
      new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0., 0.)); // TODO: Change constants



  public SUB_Pivot(SparkAbsoluteEncoder rotateEncoder) {
    config.follow(primary);
    config.inverted(true);
    secondary.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    config.inverted(false);
    config.disableFollowerMode();


    this.rotateEncoder = rotateEncoder;

    // config.absoluteEncoder.positionConversionFactor(2 * Math.PI); TODO: PUT THIS IN THE RIGHT SUBSYSTEM
    // config.absoluteEncoder.velocityConversionFactor(2 * Math.PI / 60); TODO: PUT THIS IN THE RIGHT SUBSYSTEM

    primary.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void RunArmManual(double manual) {
    primary.set(manual);
  }

  public void RunArm(boolean hasCoral) {
    currentState =
        new TrapezoidProfile.State(rotateEncoder.getPosition(), rotateEncoder.getVelocity());
    setpoint = profile.calculate(0.02, currentState, goal);

    double ff = PivotConstants.noCoralArmFeedforward.calculate(setpoint.position, setpoint.velocity);
    if (hasCoral){
      ff = PivotConstants.coralArmFeedforward.calculate(setpoint.position, setpoint.velocity);
    }

    primary.setVoltage(voltagePID.calculate(rotateEncoder.getPosition(), setpoint.position) + ff);
  }

  public void ChangeSetpoint(double setpoint) {
    goal = new TrapezoidProfile.State(setpoint, 0);
    SmartDashboard.putNumber("Arm Target Setpoint", setpoint);
  }

  public static SUB_Pivot getInstance(SparkAbsoluteEncoder rotateEncoder) {
    if (INSTANCE == null) {
      INSTANCE = new SUB_Pivot(rotateEncoder);
    }
    return INSTANCE;
  }

  public void periodic() {}
}
