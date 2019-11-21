/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.concurrent.TimeUnit;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotMap;

import frc.robot.commands.PneumaticsCommand;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

/**
 * Add your docs here.
 */
public class PneumaticsSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public Compressor c = new Compressor(RobotMap.compressorID);

  //drive motors for final setup for placing hatch
  /*
  public WPI_TalonSRX leftMaster = new WPI_TalonSRX(RobotMap.leftMasterPort);
  public WPI_VictorSPX leftSlave = new WPI_VictorSPX(RobotMap.leftSlavePort);
  public WPI_VictorSPX leftSlave2 = new WPI_VictorSPX(RobotMap.leftSlavePort2);
  public WPI_TalonSRX rightMaster = new WPI_TalonSRX(RobotMap.rightMasterPort);
  public WPI_VictorSPX rightSlave = new WPI_VictorSPX(RobotMap.rightSlavePort);
  public WPI_VictorSPX rightSlave2 = new WPI_VictorSPX(RobotMap.rightSlavePort2);
  public DifferentialDrive drive = new DifferentialDrive(leftMaster, rightMaster);
*/
  public PneumaticsSubsystem() {
    c.setClosedLoopControl(true);
    /*
    leftMaster.enableCurrentLimit(true);
    rightMaster.enableCurrentLimit(true);
    leftMaster.configContinuousCurrentLimit(15);
    leftMaster.configPeakCurrentLimit(25, 100);
    rightMaster.configContinuousCurrentLimit(15);
    rightMaster.configPeakCurrentLimit(25, 100);
    leftSlave.follow(leftMaster);
    leftSlave2.follow(leftMaster);
    rightSlave.follow(rightMaster);
    rightSlave2.follow(rightMaster);
    */
  }

  public void open(DoubleSolenoid solenoid)  {

    
    
      solenoid.set(DoubleSolenoid.Value.kForward);
    
  

    }
    public void close(DoubleSolenoid solenoid)  {

    
    
      solenoid.set(DoubleSolenoid.Value.kReverse);
    
  

    }

    //drive
    
  

  public void assist(boolean actuate, DoubleSolenoid solenoid) {
    if (actuate == true) {
      solenoid.set(DoubleSolenoid.Value.kForward);
      //System.out.println("hi");
    } else if (actuate == false) {
      solenoid.set(DoubleSolenoid.Value.kReverse);
      //System.out.println("ey");
    } else {
      solenoid.set(DoubleSolenoid.Value.kReverse);
    }
  }

  //automatically place the hatch
  public void placeHatch(){
    //drive forward
    Robot.driveSubsystem.encodersToZero();
    Robot.driveSubsystem.encoderDriveForward(10);
    //actuate hatch mehcnaism
    close(RobotMap.HatchRelease);
    //drive backward
    Robot.driveSubsystem.encoderDriveBackward(12);
    
  }

  //automatically pick up the hatch

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new PneumaticsCommand());
  }
}
