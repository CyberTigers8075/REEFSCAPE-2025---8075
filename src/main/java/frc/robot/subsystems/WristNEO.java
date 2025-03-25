// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.NEOConfigs;
import frc.robot.Robot;
import frc.robot.RobotContainer;

import com.revrobotics.spark.SparkBase.ResetMode;

public class WristNEO extends SubsystemBase {
  /** Creates a new Arm. */
  
  private SparkMax motor5, motor6;
  private SparkClosedLoopController closedLoopController;

  private AbsoluteEncoder encoder;
  public boolean manual;
  double position;
  boolean started;
  public WristNEO() {

    motor5 = new SparkMax(Constants.wristConstants.wristMotor5, MotorType.kBrushless);
   // motor6 = new SparkMax(Constants.armConstants.armMotorID2, MotorType.kBrushless);
    

   // motor6.configure(Robot.neoConfigs.armConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    motor5.configure(Robot.neoConfigs.wristConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    closedLoopController = motor5.getClosedLoopController();
    //closedLoopController = motor6.getClosedLoopController();

    encoder = motor5.getAbsoluteEncoder();
   // encoder = motor6.getAbsoluteEncoder();


    manual = true;
    position = encoder.getPosition();
    started = true;
  }

  public void setPosition(){
    //if (started){
      if(manual){
        //Down
        if( (RobotContainer.mech.getPOV() < 90 || RobotContainer.mech.getPOV()> 270) && RobotContainer.mech.getPOV() != -1){
          //Lower Limit
          System.out.println("Lower");
          while ((encoder.getPosition() < .41)){ //.579
            closedLoopController.setReference(-500, SparkBase.ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot0);
          }
    
          //UP
        } else if ( (RobotContainer.mech.getPOV() < 270 && RobotContainer.mech.getPOV() > 90)){        
            System.out.println("Increase")  ;    
              //Check if past zero                 //Upper limit
            while  (/*!(encoder.getPosition() >= .9) || */ !(encoder.getPosition() <.27)){
              closedLoopController.setReference(500, SparkBase.ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot0);
            }
        } else {
          closedLoopController.setReference(0, SparkBase.ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot0);

        }
      }else{
        System.out.println("Wrist V: " + encoder.getVelocity());
        if( encoder.getVelocity()<0 &&encoder.getPosition()<0.16){
          closedLoopController.setReference(0, SparkBase.ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot0);

        }
      }
   /* }else{
    if(manual){
        //Down
        if( (RobotContainer.mech.getPOV() < 90 || RobotContainer.mech.getPOV()> 270) && RobotContainer.mech.getPOV() != -1){
          if (!(encoder.getPosition() > .579)){ //.579
            closedLoopController.setReference(-500, SparkBase.ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot0);
          }
    
          //UP
        } else if ( (RobotContainer.mech.getPOV() < 270 && RobotContainer.mech.getPOV() > 90)){
          if (!(encoder.getPosition() <.378)){
            closedLoopController.setReference(500, SparkBase.ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot0);
        }
        } else {
          closedLoopController.setReference(0, SparkBase.ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot0);

        }
      }else{
        System.out.println("Wrist V: " + encoder.getVelocity());
        if( encoder.getVelocity()<0 &&encoder.getPosition()<0.16){
          closedLoopController.setReference(0, SparkBase.ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot0);

        }
      }
    }*/

    SmartDashboard.putNumber("Wrist Enc Pos: ", encoder.getPosition());
    SmartDashboard.putNumber("Wrist Enc Vel: ", encoder.getVelocity());


    /*if(position < Constants.wristConstants.stow){
      position = Constants.wristConstants.stow;
    }
    if(position < 65){
      position = 65;
    }*/
    
  }

 
  
  public void switchManualOn(){
    manual = true;
  }

  public void switchManualOff(){
    manual = false;
  }

  public void switchStartedOn(){
    started = true;
  }

  public void switchStartedOff(){
    started = false;
  }
  public void changePosition(double pos){
    position = pos;
  }

  
  public void retract(){
    this.changePosition(Constants.wristConstants.stow);
  }

  public void l1(){
    closedLoopController.setReference(.5/*Constants.wristConstants.l1*/, SparkBase.ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot1);
  }

  public void l2(){
    closedLoopController.setReference(.66/*Constants.wristConstants.l2*/, SparkBase.ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot1);
  }

  public void l3(){
    closedLoopController.setReference(Constants.wristConstants.l3, SparkBase.ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot1);
  }

  public void intake(){
    closedLoopController.setReference(Constants.wristConstants.intake, SparkBase.ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot1);
  }
  public void stow(){
    closedLoopController.setReference(Constants.wristConstants.stow, SparkBase.ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Wrist Pose Value", encoder.getPosition());
  }
}

//TODO: .027