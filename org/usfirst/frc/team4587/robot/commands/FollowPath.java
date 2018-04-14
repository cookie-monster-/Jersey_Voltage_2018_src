package org.usfirst.frc.team4587.robot.commands;

import org.usfirst.frc.team4587.robot.Robot;
import org.usfirst.frc.team4587.robot.subsystems.Drive.DriveControlState;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class FollowPath extends Command {

	String m_filename;
    public FollowPath(String filename) {
    	requires(Robot.getDrive());
    	m_filename = filename;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
		Robot.getDrive().setPathFilename(m_filename);
		Robot.getDrive().startPath();
    }

    // Called repeatedly whe n this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return Robot.getDrive().getState() == DriveControlState.OPEN_LOOP;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
