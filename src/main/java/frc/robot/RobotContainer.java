package frc.robot;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.*;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants;
import choreo.auto.AutoChooser;
import choreo.auto.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    public final static Joystick driver = new Joystick(Constants.DRIVER_JOYSTICK);
    public final static Joystick mech = new Joystick(Constants.MECH_JOYSTICK);


    /* Drive Controls */
    public final static int translationAxis = XboxController.Axis.kLeftY.value;
    public final int strafeAxis = XboxController.Axis.kLeftX.value;
    public final static int rotationAxis = XboxController.Axis.kRightX.value;
    
    // Buttons
    public static Trigger mech5 = new JoystickButton(mech, 5);
    public static Trigger mech6 = new JoystickButton(mech, 6);
    public static Trigger mech7 = new JoystickButton(mech, 7);
    public static Trigger mech8 = new JoystickButton(mech, 8);
    public static Trigger mech9 = new JoystickButton(mech, 9);
    public static Trigger mech10 = new JoystickButton(mech, 10);
    public static Trigger mech11 = new JoystickButton(mech, 11);




;
    /* Driver Buttons */
    private Trigger driverA = new JoystickButton(driver, Constants.A);
    public final static JoystickButton robotCentric = new JoystickButton(driver, Constants.LB);

    /* Subsystems */
   // public static final Swerve s_Swerve = new Swerve();
    public static final Arm arm = new Arm();
    
    /* autoChooser, forced to be depracated */
   // private final SendableChooser<Command> autoChooser;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
    /*     s_Swerve.setDefaultCommand(
            new Drive(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean()
            ));
*/
        arm.setDefaultCommand(new ArmCommand(arm));
        
        
        // Configure the button bindings
        configureButtonBindings();
       
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
 //       driverA.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        mech5.onTrue(new InstantCommand(() -> arm.switchManualOn()));
        mech6.onTrue(new InstantCommand(() -> arm.switchManualOff()));
        mech7.onTrue(new InstantCommand(() -> arm.l2()));
        mech8.onTrue(new InstantCommand(() -> arm.l1()));
        mech9.onTrue(new InstantCommand(() -> arm.intake()));
        mech10.onTrue(new InstantCommand(() -> arm.l3()));
        mech11.onTrue(new InstantCommand(() -> arm.stow()));



    }

    /**)
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        
       /* TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond, 
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.Swerve.swerveKinematics); 

        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(
                new Translation2d(0,1)
               // new Translation2d(1,-1)
            ),
            new Pose2d(0,2, Rotation2d.fromDegrees(0)),
            trajectoryConfig
        );

        PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
        PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);


        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                trajectory,
                s_Swerve::getPose, 
                Constants.Swerve.swerveKinematics, 
                xController,
                yController,
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);
        


        return new SequentialCommandGroup(
                new InstantCommand(() -> s_Swerve.resetOdometry(trajectory.getInitialPose())),
                swerveControllerCommand,
                new InstantCommand(() -> s_Swerve.stopModules())
        );
    

        */
        return new  PathPlannerAuto("New Auto 2");
}}