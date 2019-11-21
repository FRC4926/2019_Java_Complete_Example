/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.EverythingElseSubsystem;

public class EverythingElseCommand extends Command {
  public EverythingElseCommand() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.everythingElseSubsystem);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
   
   Robot.everythingElseSubsystem.displayClimbEncoder();
  
   //shooter
   Robot.everythingElseSubsystem.intake(Robot.oi.stick3.getRawButton(5),Robot.oi.stick3.getRawButton(6));
   
   //elevator
   Robot.everythingElseSubsystem.elevator(-Robot.oi.stick3.getRawAxis(1));
   
   if(Robot.oi.stick.getRawButton(1)){
     Robot.everythingElseSubsystem.placeHatch();
   }

   //climber
   Robot.everythingElseSubsystem.autoClimbEnabled = false;
   if(Robot.oi.stick3.getRawAxis(3)>0) {
     Robot.everythingElseSubsystem.climber(-Robot.oi.stick3.getRawAxis(3)); //right is down
   }
   else if(Robot.oi.stick3.getRawAxis(2)>0) {
     Robot.everythingElseSubsystem.climber(Robot.oi.stick3.getRawAxis(2)); //left is up
   }
   //else if(Robot.oi.stick3.getRawButton(4)){
     //Robot.everythingElseSubsystem.autoClimb();
   //}
   else{
     Robot.everythingElseSubsystem.climber(0);
   }

   

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
    end();
  }
}
