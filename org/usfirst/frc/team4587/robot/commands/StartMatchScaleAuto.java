package org.usfirst.frc.team4587.robot.commands;

import org.usfirst.frc.team4587.robot.Robot;
import org.usfirst.frc.team4587.robot.subsystems.Drive.DriveControlState;
import org.usfirst.frc.team4587.robot.subsystems.Intake.IntakeControlState;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class StartMatchScaleAuto extends Command {

	boolean armHasMoved;
	boolean finished;
	boolean wait;
	int count;
    public StartMatchScaleAuto() {
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
    		count++;
    	}
    	if(count>25){
    		Robot.getIntake().setIntakeControlState(IntakeControlState.IN_SLOW);
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return Robot.getDrive().getState() == DriveControlState.OPEN_LOOP;
    }

    // Called once after isFinished returns true
    protected void end() {
		//Robot.getIntake().setOutFast();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
