// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.NEOConfigs;
import frc.robot.Robot;
import frc.robot.RobotContainer;

import com.revrobotics.spark.SparkBase.ResetMode;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  
  private SparkMax motor1;
  private SparkMax motor2;
  private SparkClosedLoopController closedLoopController1, closedLoopController2;

  private SparkBaseConfig armConfig;
  private RelativeEncoder encoder;
  public boolean manual;
  double position;

  public Arm() {

    motor1 = new SparkMax(Constants.armConstants.armMotorID1, MotorType.kBrushless);
    motor2 = new SparkMax(Constants.armConstants.armMotorID2, MotorType.kBrushless);
    

    motor1.configure(Robot.neoConfigs.armConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    motor2.configure(Robot.neoConfigs.armConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    closedLoopController1 = motor1.getClosedLoopController();
    closedLoopController2 = motor2.getClosedLoopController();


    manual = false;
    position = 0;

  }

  public void setPosition(){

    
    closedLoopController1.setReference(position, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
    closedLoopController2.setReference(position, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);

    if(manual){
      double positionChange = MathUtil.applyDeadband(RobotContainer.mech.getRawAxis(Constants.stickValue), Constants.stickDeadband);
      position += positionChange;
    }

  }

  public void switchManualOn(){
    manual = true;
  }

  public void switchManualOff(){
    manual = false;
  }

  public void changePosition(double pos){
    position = pos;
  }

  public void l1(){
    this.changePosition(Constants.armConstants.l1);
  }

  public void l2(){
    this.changePosition(Constants.armConstants.l2);
  }

  public void l3(){
    this.changePosition(Constants.armConstants.l3);
  }

  public void intake(){
    this.changePosition(Constants.armConstants.intake);
  }

  public void stow(){
    this.changePosition(Constants.armConstants.stow);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
