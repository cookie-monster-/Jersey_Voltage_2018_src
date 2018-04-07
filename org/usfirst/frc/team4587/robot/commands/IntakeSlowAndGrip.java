package org.usfirst.frc.team4587.robot.commands;

import org.usfirst.frc.team4587.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class IntakeSlowAndGrip extends Command {

	boolean m_grip;
	int count;
    public IntakeSlowAndGrip(boolean grip) {
    	m_grip = grip;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	count = 0;
		Robot.getIntake().setIntakeIntake(m_grip);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	count++;
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return count>25;
    }

    // Called once after isFinished returns true
    protected void end() {
		Robot.getIntake().setInSlow();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
