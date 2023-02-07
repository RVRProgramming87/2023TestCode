// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants;
import java.util.*;
import java.util.List;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax; 
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;

import edu.wpi.first.cameraserver.CameraServer;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.TimedRobot;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.awt.image.*;
import java.awt.*;
import javax.swing.*;
import org.opencv.videoio.*;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;




/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  double dampen = .4;

  // TalonFX talonTest = new TalonFX(0);
  // VictorSPX cimTest = new VictorSPX(2);
  Joystick gamepadTest = new Joystick(0);
  
  //TalonFX motor1 = new TalonFX(1);
  //TalonFX motor2 = new TalonFX(2);
  //TalonFX motor3 = new TalonFX(8);
  //TalonFX motor4 = new TalonFX(9);

  //AHRS gyro = new AHRS(SPI.Port.kMXP);
  //ADXRS450_Gyro gyro = new ADXRS450_Gyro(); //Ethan was playing with this
  
  PhotonCamera camera;

  double heading;
  

  //AHRS gyro = new AHRS(SPI.Port.kMXP); //helios is playing with this, ignore him

  //motor1.setInverted(false);

  // CANSparkMax neoTest;

  Thread m_visionThread;

  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

  

  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    // neoTest = new CANSparkMax(1, MotorType.kBrushless);
    // // CameraServer.startAutomaticCapture();

    // VideoCapture camera = new VideoCapture(0);
    // Mat frame = new Mat();
    // camera.read(frame);
    // //use frame for image processing from here

    // Image img = Mat2BufferedImage(frame);
    // displayImage(img);

    


    // //camera garbage
    // m_visionThread =
    //     new Thread(
    //         () -> {
    //           // Get the UsbCamera from CameraServer
    //           UsbCamera camera = CameraServer.startAutomaticCapture();
    //           // Set the resolution
    //           camera.setResolution(640, 480);

    //           // Get a CvSink. This will capture Mats from the camera
    //           CvSink cvSink = CameraServer.getVideo();
    //           // Setup a CvSource. This will send images back to the Dashboard
    //           CvSource outputStream = CameraServer.putVideo("Rectangle", 640, 480);

    //           // Mats are very memory expensive. Lets reuse this Mat.
    //           Mat mat = new Mat();

    //           //helios's garbage
              
    //           // Mat matGrayTest = new Mat();
    //           // Imgproc.cvtColor(mat, matGrayTest, Imgproc.COLOR_RGB2GRAY);
    //           // matGrayTest = new Mat();

    //           // Mat matGray = new Mat();
    //           // Imgproc.cvtColor(matGrayTest, matGray, Imgproc.COLOR_GRAY2BGR);
    //           // matGray = new Mat();


    //           // This cannot be 'true'. The program will never exit if it is. This
    //           // lets the robot stop this thread when restarting robot code or
    //           // deploying.
    //           while (!Thread.interrupted()) {
    //             // Tell the CvSink to grab a frame from the camera and put it
    //             // in the source mat.  If there is an error notify the output.
    //             if (cvSink.grabFrame(mat) == 0) {
    //               // Send the output the error.
    //               outputStream.notifyError(cvSink.getError());
    //               // skip the rest of the current iteration
    //               continue;
    //             }
    //             // Put a rectangle on the image
    //             Imgproc.rectangle(
    //                 mat, new Point(100, 100), new Point(400, 400), new Scalar(255, 255, 255), 5);
    //             // Give the output stream a new image to display

    //             Mat matGrayTest = new Mat();
    //             Imgproc.cvtColor(mat, matGrayTest, Imgproc.COLOR_BGR2GRAY);
    //             matGrayTest = new Mat();
    //             outputStream.putFrame(matGrayTest);
    //           }
    //         });
    // m_visionThread.setDaemon(true);
    // m_visionThread.start();

    camera = new PhotonCamera("somethingstupid");
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
  public void autonomousPeriodic() {
    
  }

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
  public void teleopPeriodic() {
    //gyro.calibrate();

    // PhotonPipelineResult result = camera.getLatestResult();
    // boolean hasTargets = result.hasTargets();
    // List<PhotonTrackedTarget> targets = result.getTargets();
    // PhotonTrackedTarget target = result.getBestTarget();

    //double throttle = gamepadTest.getRawAxis(1);
    //double turnValue = gamepadTest.getRawAxis(4);

    //motor1.set(ControlMode.PercentOutput, (throttle+turnValue) * dampen);
    //motor2.set(ControlMode.PercentOutput, (throttle+turnValue) * dampen);

    //motor3.set(ControlMode.PercentOutput, (throttle-turnValue) * dampen);
    //motor4.set(ControlMode.PercentOutput, -(throttle-turnValue) * dampen);

    // SmartDashboard.putNumber("Motor 1 Speed: ", motor1.getMotorOutputPercent());
    // SmartDashboard.putNumber("Motor 1 Temp: ", motor1.getTemperature());
    // SmartDashboard.putNumber("Motor 1 Voltage: ", motor1.getBusVoltage());

    // SmartDashboard.putNumber("Gyros ", gyro.getAngle()); // helios is playing with this too
    //SmartDashboard.putNumber("Yaw ", gyro.getYaw()); // helios is playing with this too
    //SmartDashboard.putNumber("Pitch ", gyro.getPitch()); // helios is playing with this too
    //SmartDashboard.putNumber("Roll ", gyro.getRoll()); // helios is playing with this too
    //SmartDashboard.putNumber("Compass ", gyro.getCompassHeading()); // helios is playing with this too
    // SmartDashboard.putNumber("Cam Yaw", target.getYaw());
    // SmartDashboard.putNumber("Cam Pitch", target.getPitch());
    // SmartDashboard.putNumber("Cam Area", target.getArea());
    // SmartDashboard.putNumber("Cam Skew", target.getSkew());


  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

}
