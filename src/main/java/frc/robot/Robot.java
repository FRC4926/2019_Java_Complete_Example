/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.cameraserver.*;
import frc.robot.commands.EmptyCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.EverythingElseSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoSink;
import edu.wpi.cscore.VideoSource.ConnectionStrategy;
import frc.robot.ADIS16448_IMU;
import com.ctre.phoenix.motorcontrol.NeutralMode;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  //public static ExampleSubsystem subsystem = new ExampleSubsystem();
  public static OI oi;
   
  public static final ADIS16448_IMU imu = new ADIS16448_IMU();
  public static DriveSubsystem driveSubsystem = new DriveSubsystem();
  public static PneumaticsSubsystem pneumaticsSubsystem = new PneumaticsSubsystem();
public static EverythingElseSubsystem everythingElseSubsystem = new EverythingElseSubsystem();
public static LimelightSubsystem limelightSubsystem = new LimelightSubsystem();
  Command autonomousCommand;
  SendableChooser<Command> chooser = new SendableChooser<>();
  UsbCamera camera1;
  UsbCamera camera2;
  //VideoSink server;

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    try {

    oi = new OI();
    chooser.setDefaultOption("Default Auto", new EmptyCommand());
    // chooser.addOption("My Auto", new MyAutoCommand());
    SmartDashboard.putData("Auto mode", chooser);

    //cameras
    camera2 = CameraServer.getInstance().startAutomaticCapture(1);
    camera1 = CameraServer.getInstance().startAutomaticCapture(0);
    
    
    camera1.setResolution(240, 135);
    camera2.setResolution(240, 135);

    camera1.setFPS(10);
    camera2.setFPS(10);

    Robot.driveSubsystem.leftMaster.setNeutralMode(NeutralMode.Brake);
    Robot.driveSubsystem.rightMaster.setNeutralMode(NeutralMode.Brake);
    Robot.everythingElseSubsystem.climber1.setNeutralMode(NeutralMode.Brake);
    Robot.everythingElseSubsystem.climber2.setNeutralMode(NeutralMode.Brake);
    Robot.everythingElseSubsystem.climber3.setNeutralMode(NeutralMode.Brake);
    Robot.everythingElseSubsystem.climber4.setNeutralMode(NeutralMode.Brake);


    //server = CameraServer.getInstance().addSwitchedCamera("switched camera");
    //camera1.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
    //camera2.setConnectionStrategy(ConnectionStrategy.kKeepOpen);

    //turn light off
    limelightSubsystem.turnLight(false);

    //set climb encoder to zero 
    everythingElseSubsystem.climbEncoderReset();
    

    }
    catch(Exception e) {
      //  Block of code to handle errors
    }
   
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    try {
      //  Block of code to try
    }
    catch(Exception e) {
      //  Block of code to handle errors
    }
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   * You can use it to reset any subsystem information you want to clear when
   * the robot is disabled.
   */
  @Override
  public void disabledInit() {
    try {
      limelightSubsystem.turnLight(false);
      Robot.driveSubsystem.leftMaster.setNeutralMode(NeutralMode.Coast);
      Robot.driveSubsystem.rightMaster.setNeutralMode(NeutralMode.Coast);
      Robot.everythingElseSubsystem.climber1.setNeutralMode(NeutralMode.Coast);
      Robot.everythingElseSubsystem.climber2.setNeutralMode(NeutralMode.Coast);
      Robot.everythingElseSubsystem.climber3.setNeutralMode(NeutralMode.Coast);
      Robot.everythingElseSubsystem.climber4.setNeutralMode(NeutralMode.Coast);

      //  Block of code to try
    }
    catch(Exception e) {
      //  Block of code to handle errors
    }
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString code to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons
   * to the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {
    autonomousCommand = chooser.getSelected();
    limelightSubsystem.turnLight(true);

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (autonomousCommand != null) {
      autonomousCommand.start();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    limelightSubsystem.turnLight(true);
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }

    //set climb encoder to zero 
    everythingElseSubsystem.climbEncoderReset();
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {

    try {
      Scheduler.getInstance().run();
    }
    catch(Exception e) {
      //  Block of code to handle errors
    }
   
  
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
   
}
