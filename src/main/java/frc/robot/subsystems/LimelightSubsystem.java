/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.subsystems;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.commands.LimelightCommand;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.networktables.*;

import java.lang.Math;


import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class LimelightSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();


  private boolean m_LimelightHasValidTarget = false;
  private double m_LimelightDriveCommand = 0.0;
  private double m_LimelightSteerCommand = 0.0;
  boolean auto = false;
  public NetworkTable limelight =  NetworkTableInstance.getDefault().getTable("limelight");
  private static NetworkTableInstance table = null;
  private double drive_cmd;
  double nearest;
double angle = 0;
  public void correctAngle() {
    angle = 2*Math.cos(Math.toRadians(45))*Robot.imu.getAngleX();
//int angle = 5;
    if ((Math.abs(90 - angle) < 45)) {
      nearest = 90;
    } else if ((Math.abs(180 - angle) < 45)) {


      nearest = 180;
    } else if ((Math.abs(270 - angle) < 45)) {

      nearest = 270;
    } else if ((Math.abs(360 - angle) < 45)) {

      nearest = 0;
    }

    if (angle < (nearest -4)) {
     
        Robot.driveSubsystem.manualDrive(0.55, -0.55);

      
    } else if (angle > (nearest+4)) {
      Robot.driveSubsystem.manualDrive(-0.55, 0.55);

    } else {

    }

  }
public void printValues(){
  //System.out.println(limelight.getEntry("tv").getDouble(0));
  //System.out.println(limelight.getEntry("ta").getDouble(0));
  System.out.println("Short" + limelight.getEntry("tshort").getDouble(0));
  System.out.println("Long" + limelight.getEntry("tlong").getDouble(0));
}

public void a(Joystick stick2) {

  Update_Limelight_Tracking();

  //System.out.println("a running");
  auto = stick2.getRawButton(1); //trigger on right joystick



  if (auto)
  {
    //System.out.println("Button pressed");
    System.out.println("Valid target status: " + m_LimelightHasValidTarget);

    if (m_LimelightHasValidTarget)
    {
      //System.out.println("Limelight recieved valid target");
      Robot.driveSubsystem.drive.arcadeDrive(m_LimelightDriveCommand,m_LimelightSteerCommand);
          
    }
    else
    {

      Robot.driveSubsystem.drive.arcadeDrive(0.0,0.0);
    }
 
   // m_Drive.arcadeDrive(drive,steer);
  }
}

public void Update_Limelight_Tracking() {


        // These numbers must be tuned for your Robot!  Be careful!
        final double STEER_K = 0.04;
                           // how hard to turn toward the target
        final double DRIVE_K = 0.2;                    // how hard to drive fwd toward the target
        final double DESIRED_TARGET_AREA = 18.0;        // Area of the target when the robot reaches the wall
        final double MAX_DRIVE = 0.7;                   // Simple speed limit so we don't drive too fast

        double validTarget = limelight.getEntry("tv").getDouble(0);
        double limelightX = limelight.getEntry("tx").getDouble(0); //x value
        double limelightY = limelight.getEntry("ty").getDouble(0); //y value
        double limelightArea = limelight.getEntry("ta").getDouble(0);
        double tshort = limelight.getEntry("tshort").getDouble(0); //skew
        double tlong = limelight.getEntry("tlong").getDouble(0); //skew
        double ta0 = limelight.getEntry("ta0").getDouble(0);

        double thoriz = limelight.getEntry("thoriz").getDouble(0);
        double tvert = limelight.getEntry("tvert").getDouble(0);


        if (validTarget < 1.0)
        {
          m_LimelightHasValidTarget = false;
          m_LimelightDriveCommand = 0.0;
          m_LimelightSteerCommand = 0.0;
          return;
        }

        //System.out.println("thoriz " + thoriz);
        //System.out.println("tvert " + tvert);

        m_LimelightHasValidTarget = true;

        // Start with proportional steering
        double steer_cmd = limelightX*STEER_K;
       // double steer_cmd = 88+ts))/2 * STEER_K;
        m_LimelightSteerCommand = steer_cmd;

        // try to drive forward until the target area reaches our desired area
        if(limelightArea>DESIRED_TARGET_AREA){
          drive_cmd = 0;
        }
        else{
          drive_cmd = (DESIRED_TARGET_AREA - limelightArea) * DRIVE_K;
        }
        

        // don't let the robot drive too fast into the goal
        if (drive_cmd > MAX_DRIVE)
        {
          drive_cmd = MAX_DRIVE;
        }
        m_LimelightDriveCommand = drive_cmd;
  
  }

  private static NetworkTableEntry getValue(String key) {
    if(table==null){
      table = NetworkTableInstance.getDefault();
    }

    return table.getTable("limelight").getEntry(key);
      
  }

  //turn on light on limelight
  public void turnLight(boolean command){
    if(command==true){
      getValue("ledMode").setNumber(0);
      //System.out.println("light on");
    }
    else if(command==false){






































      getValue("ledMode").setNumber(1);
      //System.out.println("light off");
    }
  }

  //turn off light on lime light 

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new LimelightCommand());
    
  }
}





/*




*/