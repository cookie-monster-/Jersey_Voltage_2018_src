package org.usfirst.frc.team4587.robot.commands;

import org.usfirst.frc.team4587.robot.Constants;
import org.usfirst.frc.team4587.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class IntakeAuto extends Command {

    public IntakeAuto() {
    }

    boolean finished;
    int count;
    int count1;
    // Called just before this Command runs the first time
    protected void initialize() {
		Robot.getIntake().setIntake();
		Robot.getIntake().setIntakeGrip(true);
		finished = false;
		count=0;
		count1=0;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if(Robot.getIntake().getUltraInches()<=Constants.kIntakeCubeDistInches){
    		count++;
    		if(count>5){
    			finished = true;
    	    	Robot.getIntake().setIntakeGrip(false);
    		}
    	}else{
    		count=0;
    	}
    	if(finished){
    		count1++;
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return count1>25;
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
