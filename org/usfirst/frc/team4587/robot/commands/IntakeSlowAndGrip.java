package org.usfirst.frc.team4587.robot.commands;

import org.usfirst.frc.team4587.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class IntakeSlowAndGrip extends Command {

	boolean m_grip;
    public IntakeSlowAndGrip(boolean grip) {
    	m_grip = grip;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
		Robot.getIntake().setInSlow();
		Robot.getIntake().setIntakeGrip(m_grip);
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
    }
}
