/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.PneumaticsSubsystem;
import java.lang.Object.*;

public class PneumaticsCommand extends Command {
  public PneumaticsCommand() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.pneumaticsSubsystem);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
 
    
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    
    //place the hatch
    if(Robot.oi.stick.getRawButton(1)){
      Robot.pneumaticsSubsystem.placeHatch();
      System.out.println("hello");
    }

      //actuate the hatch mehcanism
    if (Robot.oi.stick3.getRawButton(3)){
      Robot.pneumaticsSubsystem.open(RobotMap.HatchRelease);
    } 

    if(Robot.oi.stick3.getRawButton(2)){
      Robot.pneumaticsSubsystem.close(RobotMap.HatchRelease);
    }
   
     //frog assist for the climb      
    if((Math.abs(Robot.oi.stick.getY()) < 0.15) && (Math.abs(Robot.oi.stick2.getY()) < 0.15) ){
      if(Robot.oi.stick3.getRawButton(4)==false){
        Robot.pneumaticsSubsystem.assist(Robot.oi.stick3.getRawButton(1),RobotMap.FrogAssist);
      }
    } 


    
/*

      //final placement for hatch
      if (Robot.oi.stick.getRawButton(1)){
        int count =0;
        //drive forward
        while(count<20000){
          Robot.pneumaticsSubsystem.drive(-0.6,-0.6);
          count++;
        }
        //release
        Robot.pneumaticsSubsystem.close(RobotMap.HatchRelease);
        //drive backward
        count=0;
        while(count<60000){
          Robot.pneumaticsSubsystem.drive(0.5,0.5);
          count++;
        }
      }

      */
  }
  

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
   // end();
  }
}
