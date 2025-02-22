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
    public static final int stickValue = 1;
    
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

    public static final class wristConstants{
         
        /* Module Gear Ratios */
        public static final double wristGearRatio = (1.0);

        /** Radians per Second */
        public static final double maxWristVelocity = 10.0; //TODO: This must be tuned to specific robot

        /* Motor Inverts */
        public static final InvertedValue wristMotorInvert = InvertedValue.CounterClockwise_Positive;

        /* Wrist Encoder Invert */
        public static final SensorDirectionValue cancoderInvert = SensorDirectionValue.CounterClockwise_Positive;

        /* Wrist Current Limiting */
        public static final int wristCurrentLimit = 25;
        public static final int wristCurrentThreshold = 40;
        public static final double wristCurrentThresholdTime = 0.1;
        public static final boolean wristEnableCurrentLimit = true;

        /* Wrist Motor PID Values */
        public static final double wristKP = 10;
        public static final double wristKI = 0.0;
        public static final double wristKD = 0.0;

        //private final MotorOutputConfigs m_configs = new MotorOutputConfigs();

        /* Neutral Modes */
        public static final NeutralModeValue wristNeutralMode = NeutralModeValue.Coast;

        //TODO: This must be tuned to specific motor
        public static final int wristMotor1 = 9;
        public static final int wristMotor2 = 10;
        public static final int canCoderID = 0;
    }
    
    public static final class armConstants{
        public static final int armMotorID1 = 1;
        public static final int armMotorID2 = 2;
        public static final double l1 = 10;
        public static final double l2 = 20;
        public static final double l3 = 30;
        public static final double stow = 5;
        public static final double intake = 0;
    }
}
