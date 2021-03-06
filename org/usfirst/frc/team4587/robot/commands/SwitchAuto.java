package org.usfirst.frc.team4587.robot.commands;

import org.usfirst.frc.team4587.robot.Constants;
import org.usfirst.frc.team4587.robot.Robot;
import org.usfirst.frc.team4587.robot.subsystems.Intake.IntakeControlState;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class SwitchAuto extends CommandGroup {

    public SwitchAuto() {
    	String gm = "RRR";//Robot.getGm();
    	CommandGroup firstLiftControl = new CommandGroup();
    	CommandGroup firstStep = new CommandGroup();
    	CommandGroup secondLiftControl = new CommandGroup();
    	CommandGroup secondMotion = new CommandGroup();
    	CommandGroup secondStep = new CommandGroup();
    	
    	String filename1="";
    	String filename2="";
    	String filename3 = "centerToPyramid";
    	if(gm.startsWith("L")){
			filename1 = "centerToLeftSwitch";
			filename2 = "leftSwitchToCenter";
		}else if(gm.startsWith("R")){
			filename1 = "centerToRightSwitch";
			filename2 = "rightSwitchToCenter";
		}

    	firstLiftControl.addSequential(new SetIntakeState(IntakeControlState.MANUAL_IN));
    	firstLiftControl.addSequential(new DelayTime(0.1));
    	firstLiftControl.addSequential(new SetLiftArmSetpointsWait(Constants.kLiftFlooperHeight,Constants.kArmFlooperDeg));
    	firstLiftControl.addSequential(new DelayPathPos(3));
    	firstLiftControl.addSequential(new SetIntakeState(IntakeControlState.OUT_FAST));

    	firstStep.addParallel(new FollowPath(filename1));
    	firstStep.addParallel(firstLiftControl);

    	secondLiftControl.addSequential(new DelayTime(0.5));
    	secondLiftControl.addSequential(new SetLiftArmSetpoints(Constants.kLiftSoftStopLow,Constants.kArmIntakeDeg));
    	secondLiftControl.addSequential(new DelayTime(0.5));
    	secondLiftControl.addSequential(new SetIntakeState(IntakeControlState.INTAKE));

    	secondMotion.addSequential(new FollowPath(filename2));
    	secondMotion.addSequential(new FollowPath(filename3));

    	secondStep.addParallel(secondLiftControl);
    	secondStep.addParallel(secondMotion);
    	
    	addSequential(firstStep);
    	addSequential(secondStep);

    	/*addSequential(new FollowPath(filename1));
    	addSequential(new FollowPath(filename2));
    	addSequential(new FollowPath(filename3));*/
    	//addSequential(new FollowPath("pyramidToRightScale"));
    }
    protected void initialize(){
    	
    }
}
