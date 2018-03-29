package org.usfirst.frc.team4587.robot.commands;

import org.usfirst.frc.team4587.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class SetLiftScale extends Command {

	double m_height=3.1;
	double m_degrees=-170;
	double m_endDegrees=-180;
    public SetLiftScale() {
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	/*if(Robot.getLift().isScaleHigh()){
    		m_endDegrees = -135;
    	}else{
    		m_endDegrees = -180;
    	}*/
    	Robot.getArm().setSetpoint(m_degrees);
    	Robot.getArm().startPath();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if(Math.abs(m_degrees - Robot.getArm().getDCurrent()) < 4){
        	//Robot.getLift().setSetpoint(m_height);
        	//Robot.getLift().startPath();
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return true;//Math.abs(m_height - Robot.getLift().getDCurrent()) < 0.1;
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
