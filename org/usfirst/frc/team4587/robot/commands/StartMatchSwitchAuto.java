package org.usfirst.frc.team4587.robot.commands;

import org.usfirst.frc.team4587.robot.Robot;
import org.usfirst.frc.team4587.robot.subsystems.Drive.DriveControlState;
import org.usfirst.frc.team4587.robot.subsystems.Intake.IntakeControlState;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class StartMatchSwitchAuto extends Command {

    public StartMatchSwitchAuto() {
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.getLift().setLiftSetpoint(1.0);
    	Robot.getLift().startSetpoint();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if(Robot.getLift().getPos() > 0.7){
    		Robot.getIntake().setIntakeControlState(IntakeControlState.INTAKE);
        	Robot.getLift().setArmSetpoint(-170.0);
        	Robot.getLift().setLiftSetpoint(0.0);
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return Robot.getPathsRan() > 0 && Robot.getDrive().getState() == DriveControlState.OPEN_LOOP;
    }

    // Called once after isFinished returns true
    protected void end() {
		Robot.getIntake().setIntakeControlState(IntakeControlState.OUT_FAST);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
