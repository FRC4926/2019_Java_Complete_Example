/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.RobotMap;
import frc.robot.commands.DriveManuallyCommand;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

/**
 * Add your docs here.
 */
public class DriveSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public WPI_TalonSRX leftMaster = new WPI_TalonSRX(RobotMap.leftMasterPort);
  public WPI_VictorSPX leftSlave = new WPI_VictorSPX(RobotMap.leftSlavePort);
  public WPI_VictorSPX leftSlave2 = new WPI_VictorSPX(RobotMap.leftSlavePort2);
  public WPI_TalonSRX rightMaster = new WPI_TalonSRX(RobotMap.rightMasterPort);
  public WPI_VictorSPX rightSlave = new WPI_VictorSPX(RobotMap.rightSlavePort);
  public WPI_VictorSPX rightSlave2 = new WPI_VictorSPX(RobotMap.rightSlavePort2);
  
  public DifferentialDrive drive = new DifferentialDrive(leftMaster, rightMaster);

  // Hey Rishi: add that if statement later for the limelight button
  public DriveSubsystem() {
    
    leftSlave.follow(leftMaster);
    leftSlave2.follow(leftMaster);
    rightSlave.follow(rightMaster);
    rightSlave2.follow(rightMaster);
    

    leftMaster.configContinuousCurrentLimit(10);
    leftMaster.configPeakCurrentDuration(20);
    leftMaster.configPeakCurrentLimit(10);
    leftMaster.enableCurrentLimit(true);
    


    rightMaster.configContinuousCurrentLimit(10);
    rightMaster.configPeakCurrentDuration(20);
    rightMaster.configPeakCurrentLimit(10);
    rightMaster.enableCurrentLimit(true);



    //encoders
    leftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    rightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    leftMaster.setSelectedSensorPosition(0);
    rightMaster.setSelectedSensorPosition(0);

  }

  //needs to be fixed
  public void encoderDriveBackward(double inchBackward){
    double desiredTicks = inchBackward*(1/(6*3.1415))*4096;
    double leftPosition = leftMaster.getSelectedSensorPosition();
    double rightPosition = rightMaster.getSelectedSensorPosition();
    double distance = (leftPosition+Math.abs(rightPosition))/2;

    while(distance<desiredTicks){
      //System.out.println("Actual "+distance);
      //System.out.println("Desired "+desiredTicks);
      leftPosition = leftMaster.getSelectedSensorPosition();
      rightPosition = rightMaster.getSelectedSensorPosition();
      distance = (Math.abs(leftPosition)+Math.abs(rightPosition))/2;
      drive.tankDrive(-0.4, -0.4);
      }
  }

  public void encodersToZero(){
    leftMaster.setSelectedSensorPosition(0);
    rightMaster.setSelectedSensorPosition(0);
  }
  
  public void encoderDriveForward(double inchForward){
    double desiredTicks = inchForward*(1/(6*3.1415))*4096;
    double leftPosition = leftMaster.getSelectedSensorPosition();
    double rightPosition = rightMaster.getSelectedSensorPosition();
    double distance = (leftPosition+Math.abs(rightPosition))/2;

    while(distance<desiredTicks){
      //System.out.println("Actual "+distance);
      //System.out.println("Desired "+desiredTicks);
      leftPosition = leftMaster.getSelectedSensorPosition();
      rightPosition = rightMaster.getSelectedSensorPosition();
      distance = (Math.abs(leftPosition)+Math.abs(rightPosition))/2;
      drive.tankDrive(0.4, 0.4);
      }
  }

  public void manualDrive(double left, double right) {
    // don't move if within deadband
    if (Math.abs(left) < 0.1) {
      left = 0;
    }
    if (Math.abs(right) < 0.1) {
      right = 0;
    }

    //System.out.println("driving");

    // subtract deadband
    if(left<0){
      left = left+0.1;
      right = right+0.1;
    }
    else{
      left = left-0.1;
      right = right-0.1;
    }

    // cube
    left = Math.pow(left, 3);
    right = Math.pow(right, 3);

    // add deadband and multiply to get back to max of 1
    if(left<0){
      left = (left-0.1)*1.2;
      right = (right-0.1)*1.2;
    }
    else{
      left = (left+0.1)*1.2;
      right = (right+0.1)*1.2;
    }

    if (left > 1) {
      left = 1;
    }
    if (right > 1) {
      right = 1;
    }

    drive.tankDrive(-left, -right);

    // drive.tankDrive(move,turn);
  }

  public void slowDrive(double left, double right) {

    // don't move if within deadband
    if (Math.abs(left) < 0.1) {
      left = 0;
    }
    if (Math.abs(right) < 0.1) {
      right = 0;
    }

    //System.out.println("slow drive");

    // subtract deadband
    if(left<0){
      left = left+0.1;
      right = right+0.1;
    }
    else{
      left = left-0.1;
      right = right-0.1;
    }
    
    // cube
    left = Math.pow(left, 3);
    right = Math.pow(right, 3);

    // add deadband and multiply to get back to max of 1
    if(left<0){
      left = (left-0.1)*0.8;
      right = (right-0.1)*0.8;
    }
    else{
      left = (left+0.1)*0.8;
      right = (right+0.1)*0.8;
    }

    drive.tankDrive(-left, -right);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new DriveManuallyCommand());
  }
}
