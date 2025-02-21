// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.NEOConfigs;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  
  private SparkMax motor;
  private SparkClosedLoopController closedLoopController;

  public Arm() {

    motor = new SparkMax(1, MotorType.kBrushless);
   // armConfig = NEOConfigs.armConfig;


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
