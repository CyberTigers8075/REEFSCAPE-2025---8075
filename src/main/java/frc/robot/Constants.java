package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.SwerveModuleConstants;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;


public final class Constants {
    public static final double stickDeadband = 0.1;

    //Joysticks
    public static final int DRIVER_JOYSTICK = 0;
    public static final int MECH_JOYSTICK = 1;

    // Buttons
    public static final int LEFT_JOYSTICK = 1;
    public static final int LT = 2;
    public static final int RT = 3;
    public static final int RIGHT_JOYSTICK = 5;
    public static final int A = 1;
    public static final int B = 2;
    public static final int X = 3; 
    public static final int Y = 4; 
    public static final int LB = 5; 
    public static final int RB = 6;
    public static final int BACK = 7; 
    public static final int START = 8;
    public static final int DPAD_UP = 0;
    public static final int DPAD_DOWN = 180;
    
    public static final class Swerve {
        public static final int navx = 1;

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(25); 
        public static final double wheelBase = Units.inchesToMeters(23.5);
        public static final double wheelCircumference = Units.inchesToMeters(4.0)* Math.PI;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = (6.75 / 1.0);
        public static final double angleGearRatio = (12.8 / 1.0);

        /* Motor Inverts */
        public static final InvertedValue angleMotorInvert = InvertedValue.CounterClockwise_Positive;
        public static final InvertedValue rightDriverInverted = InvertedValue.Clockwise_Positive;
        public static final InvertedValue leftDriverInverted = InvertedValue.CounterClockwise_Positive;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert = SensorDirectionValue.CounterClockwise_Positive;

        /* Swerve Current Limiting */
        public static final int angleCurrentLimit = 25;
        public static final int angleCurrentThreshold = 40;
        public static final double angleCurrentThresholdTime = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveCurrentLimit = 35;
        public static final int driveCurrentThreshold = 60;
        public static final double driveCurrentThresholdTime = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = 24.0;
        public static final double angleKI = 0.0;
        public static final double angleKD = 0.1;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.12; //TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public static final double driveKS = 0.32; //TODO: This must be tuned to specific robot
        public static final double driveKV = 1.51;
        public static final double driveKA = 0.27;

        // Theta PID
        public static final double thetaKP = 0.12; //was 0.12
        public static final double thetaKI = 0.0; //0.0
        public static final double thetaKD = 0.0; //0.0

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 7.5; //TODO: This must be tuned to specific robot
        /** Radians per Second */
        public static final double maxAngularVelocity = 5.0; //TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 4;
            public static final int angleMotorID = 3;
            public static final int canCoderID = 0;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(0.54
            );    
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 1;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(.88
            );;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 { //TODO: This must be tuned to specific robot                                                                                                                                                                                                                                                                                                                
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 2;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(0.33
            );
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 3;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(.21);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

    public static final class AutoConstants { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 1;
        public static final double kMaxAccelerationMetersPerSecondSquared = 1;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class CTREConfigs {
        public static TalonFXConfiguration swerveAngleFXConfig = new TalonFXConfiguration();
        public static TalonFXConfiguration swerveDriveFXConfig_right = new TalonFXConfiguration();
    

        public CANcoderConfiguration swerveCANcoderConfig = new CANcoderConfiguration();

        public CTREConfigs(){
            /** Swerve CANCoder Configuration */
            swerveCANcoderConfig.MagnetSensor.SensorDirection = Constants.Swerve.cancoderInvert;

            /** Swerve Angle Motor Configurations */
            /* Motor Inverts and Neutral Mode */
            swerveAngleFXConfig.MotorOutput.Inverted = Constants.Swerve.angleMotorInvert;
            swerveAngleFXConfig.MotorOutput.NeutralMode = Constants.Swerve.angleNeutralMode;

            /* Gear Ratio and Wrapping Config */
            swerveAngleFXConfig.Feedback.SensorToMechanismRatio = Constants.Swerve.angleGearRatio;
            swerveAngleFXConfig.ClosedLoopGeneral.ContinuousWrap = true;
                
            /* Current Limiting */
            swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.Swerve.angleEnableCurrentLimit;
            swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimit = Constants.Swerve.angleCurrentLimit;
            swerveDriveFXConfig_right.CurrentLimits.SupplyCurrentLowerLimit = Constants.Swerve.driveCurrentThreshold;
            swerveDriveFXConfig_right.CurrentLimits.SupplyCurrentLowerTime = Constants.Swerve.driveCurrentThresholdTime;
            // BOOKMARK: SupplyLowerLimit and LowerTime were changed effectiev 1/15/25

            /* PID Config */
            swerveAngleFXConfig.Slot0.kP = Constants.Swerve.angleKP;
            swerveAngleFXConfig.Slot0.kI = Constants.Swerve.angleKI;
            swerveAngleFXConfig.Slot0.kD = Constants.Swerve.angleKD;

            /** Swerve Drive Motor Configuration */
            /* Motor Inverts and Neutral Mode */
            
            swerveDriveFXConfig_right.MotorOutput.Inverted = Constants.Swerve.rightDriverInverted;
            swerveDriveFXConfig_right.MotorOutput.NeutralMode = Constants.Swerve.driveNeutralMode;

            /* Gear Ratio Config */
            
            swerveDriveFXConfig_right.Feedback.SensorToMechanismRatio = Constants.Swerve.driveGearRatio;

            /* Current Limiting */
            
            swerveDriveFXConfig_right.CurrentLimits.SupplyCurrentLimitEnable = Constants.Swerve.driveEnableCurrentLimit;
            swerveDriveFXConfig_right.CurrentLimits.SupplyCurrentLimit = Constants.Swerve.driveCurrentLimit;
            swerveDriveFXConfig_right.CurrentLimits.SupplyCurrentLowerLimit = Constants.Swerve.driveCurrentThreshold;
            swerveDriveFXConfig_right.CurrentLimits.SupplyCurrentLowerTime = Constants.Swerve.driveCurrentThresholdTime;
            // BOOKMARK: SupplyLowerLimit and LowerTime were changed effectiev 1/15/25

            /* PID Config */
            swerveDriveFXConfig_right.Slot0.kP = Constants.Swerve.driveKP;
            swerveDriveFXConfig_right.Slot0.kI = Constants.Swerve.driveKI;
            swerveDriveFXConfig_right.Slot0.kD = Constants.Swerve.driveKD;


            /* Open and Closed Loop Ramping */
            
            swerveDriveFXConfig_right.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = Constants.Swerve.openLoopRamp;
            swerveDriveFXConfig_right.OpenLoopRamps.VoltageOpenLoopRampPeriod = Constants.Swerve.openLoopRamp;

            
            swerveDriveFXConfig_right.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = Constants.Swerve.closedLoopRamp;
            swerveDriveFXConfig_right.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.Swerve.closedLoopRamp;
        }
    }

    //Pneumatic Intake Values
    public static final int MODULE_NUMBER = 1;
    
    public static final int FORWARD_CHANNEL_ONE = 0;
    public static final int REVERSE_CHANNEL_ONE = 1;
    
    public static final int FORWARD_CHANNEL_TWO = 2;
    public static final int REVERSE_CHANNEL_TWO = 3;

    //Intake Values
    public static final int INTAKE_MOTOR = 8;

    //Climber Values
    public static final int CLIMBER_MOTOR = 7;
    
   
    
    }
