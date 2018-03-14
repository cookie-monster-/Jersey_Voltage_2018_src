package org.usfirst.frc.team4587.robot.commands;

import org.usfirst.frc.team4587.robot.Robot;
import org.usfirst.frc.team4587.robot.subsystems.Drive.DriveControlState;

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
    	Robot.getLift().setSetpoint(1);
    	Robot.getLift().startPath();
    	armHasMoved = false;
    	finished = false;
    	wait = false;
    	count = 0;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	SmartDashboard.putNumber("liftDCurrent", Robot.getLift().getDCurrent());
    	if(Robot.getLift().getDCurrent() > 0.7 && armHasMoved == false){
    		Robot.getIntake().setIntake();
        	Robot.getArm().setSetpoint(-170);
        	Robot.getArm().startPath();
        	armHasMoved = true;
    	}
    	if(Robot.getArm().getDCurrent() < -45 && armHasMoved == true){
        	Robot.getLift().setSetpoint(0.0);
        	Robot.getLift().startPath();
    		Robot.getIntake().setInSlow();
    	}
    	if(Robot.getDrive().getState() == DriveControlState.OPEN_LOOP && armHasMoved == true){
        	Robot.getLift().setSetpoint(3.1);
        	Robot.getLift().startPath();
        	Robot.getArm().setSetpoint(-135);
    	}
    	if(Robot.getLift().getDCurrent() > 3.0){
    		Robot.getIntake().setOutFast();
    		finished = true;
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return finished;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
