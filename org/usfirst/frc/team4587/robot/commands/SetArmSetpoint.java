package org.usfirst.frc.team4587.robot.commands;

import org.usfirst.frc.team4587.robot.Constants;
import org.usfirst.frc.team4587.robot.Robot;
import org.usfirst.frc.team4587.robot.subsystems.Lift.LiftControlState;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class SetArmSetpoint extends Command {

	double m_degrees;
    public SetArmSetpoint(double degrees) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.getLift());
    	m_degrees = degrees;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	if(Robot.getLift().getClimbMode()){
    		m_degrees = Constants.kArmClimbDeg;
    	}
    	Robot.getLift().setArmSetpoint(m_degrees);
    	Robot.getLift().startSetpoint();
    	Robot.getLift().setAtScale(false);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return true;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
