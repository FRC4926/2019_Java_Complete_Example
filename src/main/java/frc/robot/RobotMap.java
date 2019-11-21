/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
  // For example to map the left and right motors, you could define the
  // following variables to use with your drivetrain subsystem.
  // public static int leftMotor = 1;
  // public static int rightMotor = 2;

  // If you are using multiple modules, make sure to define both the port
  // number and the module. For example you with a rangefinder:
  // public static int rangefinderPort = 1;
  // public static int rangefinderModule = 1;
  public static int compressorID = 17;
  public static int solenoid1 = 1;
  public static int pneumaticassist1 = 1;
  public static int pneumaticassist2 = 2;
  public static DoubleSolenoid HatchRelease = new DoubleSolenoid(RobotMap.compressorID, 0, 1);
    //DoubleSolenoid HatchTilt = new DoubleSolenoid(RobotMap.compressorID, 2, 3);
    //DoubleSolenoid ThirdStage = new DoubleSolenoid(RobotMap.compressorID, 6, 7);
    public static DoubleSolenoid FrogAssist = new DoubleSolenoid(RobotMap.compressorID, 4, 5);
    //CAN
    //drive
  public static int leftMasterPort = 1;
  public static int leftSlavePort = 3;
  public static int leftSlavePort2 = 5;
  public static int rightMasterPort = 2;
  public static int rightSlavePort = 4;
  public static int rightSlavePort2 = 6;
    //other
    public static int intake1 = 9;
  
    public static int climber1 = 7;
    public static int climber2 = 8;
    public static int climber3 = 10;
    public static int climber4 = 12;
    
    public static int elevator1 = 13;
    public static int elevator2 = 14;



  //USB
  public static int joystickPort = 0;
  public static int joystickPort1 = 1;
  public static int joystickPort2 = 2;

  
}
