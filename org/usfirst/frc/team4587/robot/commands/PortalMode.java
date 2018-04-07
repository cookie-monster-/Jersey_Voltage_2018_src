package org.usfirst.frc.team4587.robot.commands;

import org.usfirst.frc.team4587.robot.Constants;
import org.usfirst.frc.team4587.robot.Robot;
import org.usfirst.frc.team4587.robot.subsystems.Intake.IntakeControlState;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class PortalMode extends Command {

    boolean finished;
    int count;
    int count1;
    public PortalMode() {
    }

    // Called just before this Command runs the first time
    protected void initialize() {
		//Robot.setPortalMode(!Robot.getPortalMode());
		Robot.getLift().setArmSetpoint(-178.0);
		Robot.getLift().setLiftSetpoint(-1.47);
		Robot.getIntake().setIntake();
		Robot.getIntake().setIntakeIntake(true);
		finished = false;
		count=0;
		count1=0;
		Robot.setPortalModeIntaking(true);
		Robot.setPortalMode(false);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if(Robot.getPortalModeIntaking()){
    		if(Robot.getIntake().getIntakeState() != IntakeControlState.INTAKE){
    			Robot.getLift().setArmSetpoint(-180.0);
    			Robot.getLift().setLiftSetpoint(-1.47);
    			Robot.getIntake().setIntake();
    			Robot.getIntake().setIntakeIntake(true);
    		}
	    	if(Robot.getIntake().getUltraInches()<=Constants.kIntakeCubeDistInches){
	    		count++;
	    		if(count>5){
	    			finished = true;
	    	    	Robot.getIntake().setIntakeIntake(false);
	    		}
	    	}else{
	    		count=0;
	    		//Robot.getIntake().setIntake();
	    		//Robot.getIntake().setIntakeGrip(true);
	    	}
	    	if(finished){
	    		count1++;
	    	}
	    	if(count1>25){
	    		Robot.getIntake().setInSlow();
        		Robot.getLift().setArmSetpoint(0.0);
        		Robot.getLift().setLiftSetpoint(0.5);
        		Robot.setPortalModeIntaking(false);
        		finished = false;
        		count=0;
        		count1=0;
	    	}
    	}//else not intaking, wait until we are
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return Robot.getPortalMode();
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
