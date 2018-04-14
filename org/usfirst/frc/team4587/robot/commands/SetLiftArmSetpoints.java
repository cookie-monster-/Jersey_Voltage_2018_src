package org.usfirst.frc.team4587.robot.commands;

import org.usfirst.frc.team4587.robot.Constants;
import org.usfirst.frc.team4587.robot.Robot;
import org.usfirst.frc.team4587.robot.subsystems.Lift.LiftControlState;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class SetLiftArmSetpoints extends Command {

	double m_height,m_degrees;
    public SetLiftArmSetpoints(double height,double degrees) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.getLift());
    	m_height = height;
    	m_degrees = degrees;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.getLift().setLiftSetpoint(m_height);
    	Robot.getLift().setArmSetpoint(m_degrees);
    	Robot.getLift().startSetpoint();
    	Robot.getLift().setAtScale(false);
		Robot.setPortalMode(true);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return (Math.abs(Robot.getLift().getPos()-m_height) <= Constants.kLiftHeightTolerance) && (Math.abs(Robot.getLift().getArmPos()-m_degrees) <= Constants.kArmDegTolerance);
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
