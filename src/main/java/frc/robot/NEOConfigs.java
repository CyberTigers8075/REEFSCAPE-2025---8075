// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.*;

/** Add your docs here. */
public class NEOConfigs {

    private SparkBaseConfig armConfig = new SparkMaxConfig();


    public NEOConfigs(){
    /* Configure the Encoder */

    armConfig.encoder
    .positionConversionFactor(1)
    .velocityConversionFactor(1);

    /* Configure the closed loop controller */

    armConfig.closedLoop
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


    /* Apply the Config to the SparkMAX */
    }
}
