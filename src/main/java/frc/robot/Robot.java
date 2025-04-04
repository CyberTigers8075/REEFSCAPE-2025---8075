// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import java.nio.file.Path;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.util.PixelFormat;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static final CTREConfigs ctreConfigs = new CTREConfigs();
  public static final NEOConfigs neoConfigs = new NEOConfigs();

  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  Thread m_visionThread;

  DutyCycleEncoder encoder = new DutyCycleEncoder(4, 360.0, 1.0);
  
  public Robot(){/* 
  m_visionThread =
      new Thread(
          () -> {
            // Create an HTTP camera. The address will need to be modified to have the correct
            // team number. The exact path will depend on the source.
            HttpCamera camera =
                new HttpCamera("HTTP Camera", "http://10.80.75.11/video/stream.mjpg");
            // Start capturing images
            CameraServer.startAutomaticCapture(camera);
            // Set the resolution
            camera.setResolution(640, 480);

            // Get a CvSink. This will capture Mats from the camera
            CvSink cvSink = CameraServer.getVideo();
            // Setup a CvSource. This will send images back to the Dashboard
            CvSource outputStream = CameraServer.putVideo("Rectangle", 640, 480);

            // Mats are very memory expensive. Lets reuse this Mat.
            Mat mat = new Mat();

            // This cannot be 'true'. The program will never exit if it is. This
            // lets the robot stop this thread when restarting robot code or
            // deploying.
            while (!Thread.interrupted()) {
              // Tell the CvSink to grab a frame from the camera and put it
              // in the source mat.  If there is an error notify the output.
              if (cvSink.grabFrame(mat) == 0) {
                // Send the output the error.
                outputStream.notifyError(cvSink.getError());
                // skip the rest of the current iteration
                continue;
              }
              // Put a rectangle on the image
              Imgproc.rectangle(
                  mat, new Point(100, 100), new Point(400, 400), new Scalar(255, 255, 255), 5);
              // Give the output stream a new image to display
              outputStream.putFrame(mat);
            }
          });
  m_visionThread.setDaemon(true);
  m_visionThread.start();*/
  }
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
   // camera.setUsbCameraPath();
    //camera.getPath();
    //camera.setVideoMode(PixelFormat.kMJPEG, 640, 480, 30);
 
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    //SmartDashboard.putNumber("Arm Encoder", encoder.get());
    //System.out.println("Here");
    /* 
    System.out.println(RobotContainer.s_Swerve.mSwerveMods[0].moduleNumber);
    System.out.println(RobotContainer.s_Swerve.mSwerveMods[0].angleEncoder.isConnected());
    System.out.println(RobotContainer.s_Swerve.mSwerveMods[1].moduleNumber);
    System.out.println(RobotContainer.s_Swerve.mSwerveMods[1].angleEncoder.isConnected());
    System.out.println(RobotContainer.s_Swerve.mSwerveMods[2].moduleNumber);
    System.out.println(RobotContainer.s_Swerve.mSwerveMods[2].angleEncoder.isConnected());
    System.out.println(RobotContainer.s_Swerve.mSwerveMods[3].moduleNumber);
    System.out.println(RobotContainer.s_Swerve.mSwerveMods[3].angleEncoder.isConnected());
    */

  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
