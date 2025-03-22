// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
//import com.revrobotics.spark.SparkLowLevel.declarativeType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.*;

/** Add your docs here. */
public class NEOConfigs {

    public SparkBaseConfig armConfig = new SparkMaxConfig();
    public SparkBaseConfig intakeConfig = new SparkMaxConfig();
    public SparkBaseConfig wristConfig = new SparkMaxConfig();

    public NEOConfigs(){
    /* Configure the Encoder */

    armConfig.encoder
    .positionConversionFactor(1)
    .velocityConversionFactor(1);

    intakeConfig.encoder
    .positionConversionFactor(1)
    .velocityConversionFactor(1);

    wristConfig.encoder
    .positionConversionFactor(1)
    .velocityConversionFactor(1);
    /* Configure the closed loop controller */

    armConfig.inverted(false).closedLoop
    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
    .p(0.0001)
    .i(0.000001)
    .d(0)
    //.velocityFF(0.004, ClosedLoopSlot.kSlot0)
    .outputRange(-1,1)
    .p(0.0008, ClosedLoopSlot.kSlot1)
    .i(0, ClosedLoopSlot.kSlot1)
    .d(0, ClosedLoopSlot.kSlot1)
    .velocityFF(.0004,ClosedLoopSlot.kSlot1)
    .outputRange(-1, 1, ClosedLoopSlot.kSlot1)
    .maxMotion.maxVelocity(1000)
    .maxAcceleration(10000)
    .allowedClosedLoopError(0.07)
    .maxVelocity(1000, ClosedLoopSlot.kSlot1)
    .maxAcceleration(10000, ClosedLoopSlot.kSlot1)
    .allowedClosedLoopError(0.0001, ClosedLoopSlot.kSlot1);
    //.maxMotion
   // .maxAcceleration(10000)
    //.maxVelocity(500)
    //.allowedClosedLoopError(.25);

    intakeConfig.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .p(1)
    .i(0)
    .d(0)
    .outputRange(-1,1)
    .p(0.001, ClosedLoopSlot.kSlot1)
    .i(0, ClosedLoopSlot.kSlot1)
    .d(0, ClosedLoopSlot.kSlot1)
    .velocityFF(1.0/5767, ClosedLoopSlot.kSlot1)
    .outputRange(-1,1, ClosedLoopSlot.kSlot1);
    //.maxMotion
   // .maxAcceleration(10000)
    //.maxVelocity(500)
    //.allowedClosedLoopError(.25);

    
 

    wristConfig.closedLoop
    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
    .p(0.13)
    .i(0.00000)
    .d(0)
    .velocityFF(0.0004)
    .outputRange(-1,1)
    .p(0.01, ClosedLoopSlot.kSlot1)
    .i(0, ClosedLoopSlot.kSlot1)
    .d(0, ClosedLoopSlot.kSlot1)
    .velocityFF(0.0001, ClosedLoopSlot.kSlot1)
    .outputRange(-1, 1, ClosedLoopSlot.kSlot1)
    .maxMotion.maxVelocity(7000)
    .maxAcceleration(10000)
    .allowedClosedLoopError(0.25)
    .maxVelocity(7000, ClosedLoopSlot.kSlot1)
    .maxAcceleration(10000, ClosedLoopSlot.kSlot1)
    .allowedClosedLoopError(0.001, ClosedLoopSlot.kSlot1);
    
    /* Apply the Config to the SparkMAX */
    }
}
