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
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.EverythingElseCommand;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 * Add your docs here.
 */
public class EverythingElseSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public WPI_TalonSRX intake1 = new WPI_TalonSRX(RobotMap.intake1);
 
  public WPI_TalonSRX climber1 = new WPI_TalonSRX(RobotMap.climber1);
  public WPI_TalonSRX climber2 = new WPI_TalonSRX(RobotMap.climber2);
  public WPI_TalonSRX climber3 = new WPI_TalonSRX(RobotMap.climber3);
  public WPI_TalonSRX climber4 = new WPI_TalonSRX(RobotMap.climber4);
  public WPI_TalonSRX elevator1 = new WPI_TalonSRX(RobotMap.elevator1);
  public WPI_TalonSRX elevator2 = new WPI_TalonSRX(RobotMap.elevator2);
  public Encoder climberEncoder = new Encoder(8,9, false, Encoder.EncodingType.k4X);
  public boolean autoClimbEnabled = false;
  public boolean pauseForClimb = true;
  double dist =0.5*3.14/1024;  // ft per pulse

public EverythingElseSubsystem() {
  //current limiting
  elevator2.follow(elevator1);
  elevator1.configContinuousCurrentLimit(10);
  elevator1.configPeakCurrentDuration(20);
  elevator1.configPeakCurrentLimit(10);
  elevator1.enableCurrentLimit(true);
  climber2.follow(climber1);
  climber1.configContinuousCurrentLimit(20);
  climber1.configPeakCurrentDuration(30);
  climber1.configPeakCurrentLimit(20);
  climber1.enableCurrentLimit(true);
  climber4.follow(climber3);
  climber3.configContinuousCurrentLimit(20);
  climber3.configPeakCurrentDuration(30);
  climber3.configPeakCurrentLimit(20);
  climber3.enableCurrentLimit(true);

  climberEncoder.setDistancePerPulse(dist);
  climberEncoder.reset();

  //climberEnocoder.configForwardSoftLimitThreshold(-10000);
  //climberEncoder.configReverseSoftLimitThreshold(400000);

}

public void climbEncoderReset(){
  climberEncoder.reset();
}

public void autoClimb(){
  double encoderCount = climberEncoder.get();
  autoClimbEnabled = true;

  //drive bakwards to line up
  //long desired = System.currentTimeMillis() + 200;
  //while(System.currentTimeMillis() < desired){
  //  Robot.driveSubsystem.manualDrive(-0.4,-0.4);
  //}

  //actuate pneumatics cylinders until encoder has reached certain point
  if(encoderCount<200000){
    Robot.pneumaticsSubsystem.assist(true,RobotMap.FrogAssist);
  }
  //run the drive motors to get on platform
  if(encoderCount>300000){
    System.out.println("hello");
    if(encoderCount>420000){
      Robot.driveSubsystem.manualDrive(0, 0);
    }
    Robot.driveSubsystem.manualDrive(0.6, 0.6);
  }
  
  //run climb effort at slow speed
  if(encoderCount<150000){
    climber(0.3);
  }
  //run climb effort at faster speed
  else if(encoderCount<300000){
    if(pauseForClimb){
      pauseClimb();
    }
    climber(0.5);
    Robot.driveSubsystem.manualDrive(0.6, 0.6);
  }
  //run climbe slow again
  else if(encoderCount<400000){
    climber(0.3);
    Robot.driveSubsystem.manualDrive(0.6, 0.6);
  }
  //don't run climber after that point
  else{
    climber(0);
    Robot.driveSubsystem.manualDrive(0, 0);
  }
  
  
}

public void pauseClimb(){
  for(int x=0; x<1000; x++){
    climber(0);
  }
  pauseForClimb = false;
}

public void displayClimbEncoder(){
  //System.out.println(climberEncoder.get());
  //System.out.println(climberEncoder.getDistance());
  SmartDashboard.putNumber("Encoder Ticks", climberEncoder.get());
	SmartDashboard.putNumber("Distance", climberEncoder.getDistance());
}

public void intake(boolean i,boolean x) {
  
  if(i) {
    intake1.set(0.7);
  }
  else if(x){
    intake1.set(-0.7);
  }

  else{
     intake1.set(0);
  }


}

public void climber(double climberEffort) {
  climber1.set(climberEffort);
  climber3.set(-climberEffort);
  SmartDashboard.putNumber("Climb Effort", climberEffort);

}

public void elevator(double elevatorEffort) {
  
  if(elevatorEffort<-0.1){
    elevator1.set(-0.15);
  } else{
  
  elevator1.set(elevatorEffort);
  }

}

public void placeHatch(){
  long desired = System.currentTimeMillis() + 300;
  while(System.currentTimeMillis() < desired){
    Robot.driveSubsystem.manualDrive(0.5,0.5);
  }

  Robot.pneumaticsSubsystem.close(RobotMap.HatchRelease);
  try{
  Thread.sleep(400);
  } catch(Exception e){



  }
  desired = System.currentTimeMillis() + 200;
  while(System.currentTimeMillis() < desired){
  Robot.driveSubsystem.manualDrive(0.5,0.5);
  }

} 








  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
     setDefaultCommand(new EverythingElseCommand());
  }
}
