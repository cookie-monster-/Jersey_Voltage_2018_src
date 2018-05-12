package org.usfirst.frc.team4587.robot.commands;

import org.usfirst.frc.team4587.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class DelayPathPosLeft extends Command {

	double m_feet;
    public DelayPathPosLeft(double feet) {
    	m_feet = feet;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly whe n this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return Math.abs(Robot.getDrive().getRightPathPos()) >= Math.abs(m_feet);
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
