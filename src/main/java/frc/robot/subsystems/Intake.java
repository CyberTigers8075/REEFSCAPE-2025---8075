// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  
  private SparkMax intakeMotor3;
  private SparkMax intakeMotor4;
  private SparkBaseConfig intakeConfig;

  public Intake() {

    intakeMotor3 = new SparkMax(Constants.intakeConstants.intakeMotorID3, MotorType.kBrushless);
   intakeMotor4 = new SparkMax(Constants.intakeConstants.intakeMotorID4, MotorType.kBrushless);
    
    intakeMotor3.configure(Robot.neoConfigs.intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  intakeMotor4.configure(Robot.neoConfigs.intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void intakeMovement(){
        if (RobotContainer.mech.getRawButton(1) && RobotContainer.mech.getRawButton(2)){
      intakeMotor3.setVoltage(-9);
   intakeMotor4.setVoltage(9);

    }  else if (RobotContainer.mech.getRawButton(1)){
      intakeMotor3.setVoltage(9);
   intakeMotor4.setVoltage(-9);
      
    } else {
      intakeMotor3.setVoltage(0);
      intakeMotor4.setVoltage(0);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
