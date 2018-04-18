package org.usfirst.frc.team4587.robot.commands;

import org.usfirst.frc.team4587.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class DeployTines extends Command {

	long m_startTime;
	long m_currentTime;
    public DeployTines() {
    	requires(Robot.getTines());
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.getTines().setMotorLevel(0.7);
    	m_startTime = System.nanoTime();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	m_currentTime = System.nanoTime();
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return (m_currentTime - m_startTime) >= (1.0 * 1000 * 1000 * 1000);
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.getTines().setMotorLevel(0.0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
