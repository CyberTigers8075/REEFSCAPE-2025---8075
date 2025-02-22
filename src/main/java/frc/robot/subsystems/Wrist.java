// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class Wrist extends SubsystemBase {
  public double wristPosition = 0;
  public TalonFX wristMotor1, wristMotor2;
  public DutyCycleEncoder wristEncoder;

  /** Creates a new Wrist. */

    /* angle motor control requests */
  private final  PositionVoltage wristPositionVoltage = new PositionVoltage(0);

  
  public Wrist() {
        //this.angleOffset = Constants.angleOffset;
    this.wristMotor1 = new TalonFX(Constants.wristConstants.wristMotor1);
    this.wristMotor2 = new TalonFX(Constants.wristConstants.wristMotor2);

    /* Angle Encoder Config */
    wristEncoder = new DutyCycleEncoder(Constants.wristConstants.canCoderID);

    /* Angle Motor Config */
    wristMotor1.getConfigurator().apply(Robot.ctreConfigs.wristFXConfig);
    wristMotor2.getConfigurator().apply(Robot.ctreConfigs.wristFXConfig);
    resetToAbsolute();
  }

       
  public void setRotation2d(Rotation2d rotation){
    wristMotor1.setControl(wristPositionVoltage.withPosition(rotation.getRotations()));
    wristMotor2.setControl(wristPositionVoltage.withPosition(rotation.getRotations()));
  }
  
  public Rotation2d getCANcoder(){
    return Rotation2d.fromRotations(wristEncoder.get());
  }  // BOOKMARK: changed from getAbsolutePosition to get()
  
  public void resetToAbsolute(){
    double absolutePosition = getCANcoder().getRotations() /* - angleOffset.getRotations()*/;
    wristMotor1.setPosition(absolutePosition);
    wristMotor2.setPosition(absolutePosition);
  }
      
  public Rotation2d getState(){
    return Rotation2d.fromRotations(wristMotor1.getPosition().getValueAsDouble());
    
  }

  public Rotation2d getPosition(){
    return Rotation2d.fromRotations(wristMotor1.getPosition().getValueAsDouble());
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
