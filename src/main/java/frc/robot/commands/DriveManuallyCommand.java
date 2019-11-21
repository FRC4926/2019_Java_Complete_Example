/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class DriveManuallyCommand extends Command {
  public DriveManuallyCommand() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.driveSubsystem);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double left = Robot.oi.stick.getY();
    double right = Robot.oi.stick2.getY();
    if(Robot.oi.stick3.getRawButton(4)==false && Robot.oi.stick2.getRawButton(6)==false && Robot.oi.stick.getRawButton(1)==false){
      if (Robot.oi.stick2.getRawButton(3)) { // right thumb button
        //System.out.println("button pressed");
        Robot.driveSubsystem.slowDrive(left, right);
      } 
      else if (Robot.oi.stick2.getRawButton(2) == false) {
        Robot.driveSubsystem.manualDrive(left, right);
      }
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
