package org.usfirst.frc.team4587.robot.commands;

import org.usfirst.frc.team4587.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class IntakeGrip extends Command {

	boolean m_grip, m_intake;
    public IntakeGrip(boolean grip, boolean intake) {
    	m_grip = grip;
    	m_intake = intake;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
	//	Robot.getIntake().setIntakeGrip(m_grip);
	//	Robot.getIntake().setIntakeIntake(m_intake);
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
