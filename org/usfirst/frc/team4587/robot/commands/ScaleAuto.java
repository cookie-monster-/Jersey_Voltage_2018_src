package org.usfirst.frc.team4587.robot.commands;

import org.usfirst.frc.team4587.robot.Constants;
import org.usfirst.frc.team4587.robot.subsystems.Intake.IntakeControlState;
import org.usfirst.frc.team4587.robot.subsystems.Lift.ScaleState;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class ScaleAuto extends CommandGroup {

    public ScaleAuto(String gm) {
    	CommandGroup firstLiftControl = new CommandGroup();
    	CommandGroup firstStep = new CommandGroup();
    	CommandGroup secondLiftControl = new CommandGroup();
    	CommandGroup secondMotion = new CommandGroup();
    	CommandGroup secondStep = new CommandGroup();
    	CommandGroup thirdLiftControl = new CommandGroup();
    	CommandGroup thirdMotion = new CommandGroup();
    	CommandGroup thirdStep = new CommandGroup();
    	CommandGroup fourthLiftControl = new CommandGroup();
    	CommandGroup fourthMotion = new CommandGroup();
    	CommandGroup fourthStep = new CommandGroup();
    	CommandGroup fifthLiftControl = new CommandGroup();
    	//CommandGroup fourthMotion = new CommandGroup();
    	CommandGroup fifthStep = new CommandGroup();
    	
    	String filename1="";
    	String filename2="";
    	String filename3 = "";
    	if(gm.equals("LRL")||gm.equals("RRR")){
			filename1 = "rightToRightScale";
			filename2 = "rightScaleToSwitchCube";
			filename3 = "switchCubeToRightScale";
		

	    	firstLiftControl.addSequential(new DelayPathPos(14.0));// TUNE THIS !!!!!
	    	firstLiftControl.addSequential(new SetLiftScale());
	    	firstLiftControl.addSequential(new DelayPathPos(23.0));// TUNE THIS !!!!!
	    	firstLiftControl.addSequential(new SetIntakeState(IntakeControlState.OUT_FAST));
	
	    	firstStep.addParallel(new FollowPath(filename1));
	    	firstStep.addParallel(firstLiftControl);
	
	    	secondLiftControl.addSequential(new DelayTime(0.5));
	    	secondLiftControl.addSequential(new SetLiftArmSetpoints(Constants.kLiftSoftStopLow,Constants.kArmIntakeDeg));
	    	secondLiftControl.addSequential(new DelayTime(0.5));
	    	secondLiftControl.addSequential(new SetIntakeState(IntakeControlState.INTAKE));
	
	    	secondMotion.addSequential(new FollowPath(filename2));
	   // 	secondMotion.addSequential(new FollowPath(filename3));
	    	//secondMotion.addSequential(new FollowPath("rightScaleToSecondSwitchCube"));
	    	///secondMotion.addSequential(new FollowPath("secondSwitchCubeToRightScale"));
	
	    	secondStep.addParallel(secondLiftControl);
	    	secondStep.addParallel(secondMotion);
	
	    	thirdLiftControl.addSequential(new SetScaleState(ScaleState.LOW_NO_FLIP));
	    	thirdLiftControl.addSequential(new SetLiftScale());
	    	thirdLiftControl.addSequential(new DelayTime(2.5));// TUNE THIS !!!!!
	    	thirdLiftControl.addSequential(new SetIntakeState(IntakeControlState.OUT_SLOW));
	
	    	thirdMotion.addSequential(new FollowPath(filename3));
	
	    	thirdStep.addParallel(thirdLiftControl);
	    	thirdStep.addParallel(thirdMotion);
	
	    	//fourthLiftControl.addSequential(new DelayTime(0.5));
	    	fourthLiftControl.addSequential(new SetLiftArmSetpoints(Constants.kLiftSoftStopLow,Constants.kArmIntakeDeg));
	    	fourthLiftControl.addSequential(new DelayTime(0.5));
	    	fourthLiftControl.addSequential(new SetIntakeState(IntakeControlState.INTAKE));
	    	
	    	fourthMotion.addSequential(new FollowPath("rightScaleToSecondSwitchCube"));
	
	    	fourthStep.addParallel(fourthMotion);
	    	fourthStep.addParallel(fourthLiftControl);
	    	
	    	fifthLiftControl.addSequential(new DelayTime(1.5));
	    	fifthLiftControl.addSequential(new SetLiftScale());
	    	fifthLiftControl.addSequential(new DelayTime(2.0));
	    	fifthLiftControl.addSequential(new SetIntakeState(IntakeControlState.OUT_SLOW));
	    	
	    	fifthStep.addParallel(new FollowPath("secondSwitchCubeToRightScale"));
	    	fifthStep.addParallel(fifthLiftControl);
	    	
	    	addSequential(firstStep);
	    	addSequential(secondStep);
	    	addSequential(thirdStep);
	    	addSequential(fourthStep);
	    	addSequential(fifthStep);
    	}else if(gm.equals("LLL")||gm.equals("RLR")){
    		addSequential(new FollowPath("anyToCrossLineBackwards"));
    	}
    }
}
