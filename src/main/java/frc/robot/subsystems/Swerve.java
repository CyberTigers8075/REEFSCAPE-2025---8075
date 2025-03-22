package frc.robot.subsystems;



import frc.robot.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;


import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.studica.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    private static final SwerveModulePosition[][] SwerveModulePosition = null;
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public AHRS gyro;
    public PositionVoltage anglePosition;
    public PositionVoltage drivePosition;
    public TalonFXConfiguration swerveAngleFXConfig;
    public Translation2d trans;
    public double rot;
    public boolean fieldRelative;

    public double maxSpeed =7.5;
    public double maxAngularVelocity = 5;

    public Swerve() {
        gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);
        gyro.zeroYaw();

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getGyroYaw(), getModulePositions());

        RobotConfig config;
        try{
            config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                this::getPose, // Robot pose supplier\
                this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
                new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
                ),
                config, // The robot configuration
                () -> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
                },
                this // Reference to this subsystem to set requirements
        );

        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
        }
            // Configure AutoBuilder last
        
   
        }

    public void driveRobotRelative(ChassisSpeeds speeds){
         
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates( speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, maxSpeed);
        setModuleStates(swerveModuleStates);
        
    } 
    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        //fieldRelative = false;
        // TODO: figure if this command is relative to pathplanner
        trans = translation;
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation/2, 
                                    getHeading()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation/2)
                                );
        SmartDashboard.putNumber("pose X: ", translation.getX());
        SmartDashboard.putNumber("pose Y: ", translation.getY());
        SmartDashboard.putNumber("rotation", rotation);

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
           
        }
    }    

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, maxSpeed);
        

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public Pose2d getPose() {
        if (swerveOdometry == null) {
            return new Pose2d(0,0, new Rotation2d(0));
        }

        SmartDashboard.putNumber("Robot Location Up/Down", swerveOdometry.getPoseMeters().getX());
        SmartDashboard.putNumber("Robot Location Left/Right", swerveOdometry.getPoseMeters().getY());
        SmartDashboard.putNumber("POV Driver Angle", gyro.getAngle());


        SmartDashboard.putNumber("gyro pitch", gyro.getPitch());
        SmartDashboard.putNumber("gyro roll", gyro.getRoll());
        SmartDashboard.putNumber("pitch rate", getPitchRate());
        
        return swerveOdometry.getPoseMeters();
    
    }

    public double getPitch() {
        return gyro.getPitch();
    }

    public double getPitchRate() {
        return gyro.getRawGyroY();
    }

    public double getRoll() {
        return gyro.getRoll();
    }



    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(this.getRotation2d(), getModulePositions() , pose);
    }
    
    public void setPose(Pose2d pose) {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }
   

    //sets the robot positions and direction to the spcified position
    public void resetPose(Pose2d pose){
        if (swerveOdometry == null) {
            return;
        }
        swerveOdometry.resetPosition(resetGyroYaw(), getModulePositions(), pose);
        
    }

    public Rotation2d getHeading(){
        return getPose().getRotation();
    }

    public Rotation2d getRotation2d() {
        return getHeading();
    }

    public void setHeading(Rotation2d heading){
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading(){
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    public void zeroGyro(){
         // this will zero the gyros of the motors
        gyro.reset();
        double rotationDegrees = 0;
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && (alliance.get() == DriverStation.Alliance.Red)) {
            rotationDegrees = 180;
        }

        Pose2d oldPose = getPose();
        Pose2d pose = new Pose2d(oldPose.getX(), oldPose.getY(), Rotation2d.fromDegrees(rotationDegrees));
        resetPose(pose);
    }
    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(-gyro.getYaw());
    }
    
    public Rotation2d resetGyroYaw(){
        return Rotation2d.fromDegrees(0);
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    public ChassisSpeeds getRobotRelativeSpeeds(){
        return Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());
    }


    @Override
    public void periodic(){
        if (swerveOdometry == null) {
            return;
        }


        swerveOdometry.update(getGyroYaw(), getModulePositions());

        for(SwerveModule mod : mSwerveMods){
            //SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Speed", mod.getState().speedMetersPerSecond); 
 
        }
        SmartDashboard.putNumber("Robot Field Angle (Inversed)", getHeading().getDegrees());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
    }
    
    public void stopModules() {
        this.mSwerveMods[0].mAngleMotor.stopMotor();
        this.mSwerveMods[0].mDriveMotor.stopMotor();
        this.mSwerveMods[1].mAngleMotor.stopMotor();
        this.mSwerveMods[1].mDriveMotor.stopMotor();
        this.mSwerveMods[2].mAngleMotor.stopMotor();
        this.mSwerveMods[2].mDriveMotor.stopMotor();
        this.mSwerveMods[3].mAngleMotor.stopMotor();
        this.mSwerveMods[3].mDriveMotor.stopMotor();
        
    }   

    

}