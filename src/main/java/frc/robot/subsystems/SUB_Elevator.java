// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class SUB_Elevator extends SubsystemBase {
  private static SUB_Elevator INSTANCE = null;
  private boolean HasHomed = false;
  private static SparkMax primary = new SparkMax(0,MotorType.kBrushless);
  private static SparkMax secondary = new SparkMax(0, MotorType.kBrushless);
  private static SparkMaxConfig config = new SparkMaxConfig();
  private static RelativeEncoder primaryencoder = primary.getEncoder();
  
private SUB_Elevator(){
  
  config.inverted(true);
}

public static void RunElevatorManual(double manual){

}

public static void RunElevator(){

}

public static SUB_Elevator getInstance() {
  if (INSTANCE == null) {
    INSTANCE = new SUB_Elevator();
  }

  return INSTANCE;
}

public void periodic(){}
}