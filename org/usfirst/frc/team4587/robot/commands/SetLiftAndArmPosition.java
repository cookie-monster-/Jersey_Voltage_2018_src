package org.usfirst.frc.team4587.robot.commands;

import org.usfirst.frc.team4587.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class SetLiftAndArmPosition extends Command {

	double m_height;
	double m_degrees;
	double m_endDegrees;
    public SetLiftAndArmPosition(double height,double degrees, double endDegrees) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	m_height = height;
    	m_degrees = degrees;
    	m_endDegrees = endDegrees;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.getArm().setSetpoint(m_degrees);
    	Robot.getArm().startPath();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if(Math.abs(m_degrees - Robot.getArm().getDCurrent()) < 4){
        	Robot.getLift().setSetpoint(m_height);
        	Robot.getLift().startPath();
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return Math.abs(m_height - Robot.getLift().getDCurrent()) < 0.1;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.getArm().setSetpoint(m_endDegrees);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
