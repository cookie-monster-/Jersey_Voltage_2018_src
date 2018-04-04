package org.usfirst.frc.team4587.robot.commands;

import org.usfirst.frc.team4587.robot.Robot;
import org.usfirst.frc.team4587.robot.subsystems.Intake.IntakeControlState;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class IntakeMotors extends Command {

	IntakeControlState m_state;
    public IntakeMotors(IntakeControlState state) {
    	m_state = state;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.getIntake().setIntakeControlState(m_state);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
