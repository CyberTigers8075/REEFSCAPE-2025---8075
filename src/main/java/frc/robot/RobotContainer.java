package frc.robot;


import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.RobotContainer;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    public final static XboxController driver = new XboxController(Constants.DRIVER_JOYSTICK);
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

    /* Driver Buttons */
    private Trigger driverA = new JoystickButton(driver, Constants.A);
    public final static JoystickButton robotCentric = new JoystickButton(driver, Constants.LB);

    /* Subsystems */
    static final Swerve s_Swerve = new Swerve();
    public static final Arm arm = new Arm();
    public static final WristNEO wrist = new WristNEO();
    public static final Intake intake = new Intake();
    
    /* autoChooser, forced to be depracated */
   // private final SendableChooser<Command> autoChooser;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
          s_Swerve.setDefaultCommand(
            new Drive(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean()
            )); 
            
        arm.setDefaultCommand(new ArmCommand(arm));
        wrist.setDefaultCommand(new WristCommand(wrist));
        intake.setDefaultCommand(new IntakeMovement(intake));
        
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
        //driverA.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        /* Arm Bindings */
        
        mech5.onTrue(new InstantCommand(() -> arm.switchManualOn()));
        mech6.onTrue(new InstantCommand(() -> arm.switchManualOff()));
        //mech7.onTrue(new InstantCommand(() -> arm.l2()));
       //mech8.onTrue(new InstantCommand(() -> arm.l1()));
        //mech9.onTrue(new InstantCommand(() -> arm.intake()));
        //mech10.onTrue(new InstantCommand(() -> arm.l3()));
        //mech11.onTrue(new InstantCommand(() -> arm.stow()));
        
       /*  Wrist Bindings */
       mech5.onTrue(new InstantCommand(() -> wrist.switchManualOn()));
       mech6.onTrue(new InstantCommand(() -> wrist.switchManualOff()));
        mech7.onTrue(new InstantCommand(() -> wrist.l2()));
       mech8.onTrue(new InstantCommand(() -> wrist.l1()));
       //mech9.onTrue(new InstantCommand(() -> wrist.intake()));*/
        //mech10.onTrue(new InstantCommand(() -> wrist.l3()));
        //mech11.onTrue(new InstantCommand(() -> wrist.stow()));
    
    }

    /**)
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return new  PathPlannerAuto("New New Auto");
}}