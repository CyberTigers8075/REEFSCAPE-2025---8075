// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
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
import frc.robot.Constants.armConstants;
import frc.robot.subsystems.WristNEO;
import com.revrobotics.spark.SparkBase;


import com.revrobotics.spark.SparkBase.ResetMode;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  
  private SparkMax motor1;
  private SparkMax motor2;
  private SparkClosedLoopController closedLoopController1, closedLoopController2;

  private AbsoluteEncoder encoder;
  public boolean manual;
  //double position;
  public String level;

  public Arm() {
      
    motor1 = new SparkMax(Constants.armConstants.armMotorID1, MotorType.kBrushless);
    //motor2 = new SparkMax(Constants.armConstants.armMotorID2, MotorType.kBrushless);
    

    motor1.configure(Robot.neoConfigs.armConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    //motor2.configure(Robot.neoConfigs.armConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    closedLoopController1 = motor1.getClosedLoopController();
    //closedLoopController2 = motor2.getClosedLoopController();


    encoder = motor1.getAbsoluteEncoder();
    manual = false;
    //position = 0;
    level = "stow";
  }

  public void setPosition(){
    if(manual){
      double velocity = MathUtil.applyDeadband(RobotContainer.mech.getRawAxis(1), 0.1)  * 500;
      if(velocity == 0){
        motor1.setVoltage(-0.18);
      } else {
        if(encoder.getPosition() > 0.65 && velocity > 0){
          motor1.setVoltage(-0.18);
        } else if(encoder.getPosition() < .4 && velocity < 0){
          motor1.setVoltage(-0.18);
        } else {
        closedLoopController1.setReference(-velocity, SparkBase.ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot0);
        }
      }
    //  System.out.println("sent:"+ velocity);
    //  System.out.println("actual" + encoder.getVelocity());
    //  System.out.println(encoder.getPosition());
    }

    System.out.println("\n");

    
  }

 public void armToWristLimit(){
    /*if(60 < position){
      RobotContainer.wrist.retract();
    }
    if(position < 120){
      RobotContainer.wrist.retract();
    }
    if(60 < position && 120 > position){
      RobotContainer.wrist.stow();
    }else if(!manual){
        if (level == "l1"){
            RobotContainer.wrist.l1();
        }
        if (level == "l2"){
          RobotContainer.wrist.l2();
        }
        if (level == "l3"){
          RobotContainer.wrist.l3();
        }
        if (level == "intake"){
          RobotContainer.wrist.intake();
        }
      }
   */
  }   


  

  public void switchManualOn(){
    manual = true;
  }

  public void switchManualOff(){
    manual = false;
  }

  public void changePosition(double pos){
    //position = pos;
  }

  public void l1(){
    closedLoopController1.setReference(Constants.armConstants.l1, SparkBase.ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot1);
  }

  public void l2(){
    closedLoopController1.setReference(Constants.armConstants.l2, SparkBase.ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot1);
  }

  public void l3(){
    closedLoopController1.setReference(Constants.armConstants.l3, SparkBase.ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot1);
  }

  public void intake(){
    closedLoopController1.setReference(Constants.armConstants.intake, SparkBase.ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot1);
  }

  public void stow(){
    closedLoopController1.setReference(Constants.armConstants.stow, SparkBase.ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot1);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
