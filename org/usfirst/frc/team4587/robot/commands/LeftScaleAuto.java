package org.usfirst.frc.team4587.robot.commands;

import org.usfirst.frc.team4587.robot.Constants;
import org.usfirst.frc.team4587.robot.subsystems.Intake.IntakeControlState;
import org.usfirst.frc.team4587.robot.subsystems.Lift.ScaleState;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class LeftScaleAuto extends CommandGroup {

    public LeftScaleAuto(String gm) {
    	CommandGroup firstLiftControl = new CommandGroup();
    	CommandGroup firstMotion = new CommandGroup();
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
    	if(gm.equals("RLR")||gm.equals("LLL")){
			filename1 = "leftToLeftScale";
			filename2 = "leftScaleToSwitchCube";
			filename3 = "switchCubeToLeftScale";
		

	    	firstLiftControl.addSequential(new DelayPathPosLeft(14.0));// TUNE THIS !!!!!
	    	firstLiftControl.addSequential(new SetLiftScale());
	    	firstLiftControl.addSequential(new DelayPathPosLeft(23.5));// TUNE THIS !!!!!
	    	firstLiftControl.addSequential(new SetIntakeState(IntakeControlState.OUT_FAST));
	
	    	firstStep.addParallel(new FollowPath(filename1));
	    	firstStep.addParallel(firstLiftControl);
	
	    	//secondLiftControl.addSequential(new DelayTime(0.5));
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
	    	thirdLiftControl.addSequential(new SetIntakeState(IntakeControlState.OUT_FAST));
	
	    	thirdMotion.addSequential(new FollowPath(filename3));
	
	    	thirdStep.addParallel(thirdLiftControl);
	    	thirdStep.addParallel(thirdMotion);
	
	    	//fourthLiftControl.addSequential(new DelayTime(0.5));
	    	fourthLiftControl.addSequential(new SetLiftArmSetpoints(Constants.kLiftSoftStopLow,Constants.kArmIntakeDeg));
	    	fourthLiftControl.addSequential(new DelayTime(0.5));
	    	fourthLiftControl.addSequential(new SetIntakeState(IntakeControlState.INTAKE));
	    	
	    	fourthMotion.addSequential(new FollowPath("leftScaleToSecondSwitchCube"));
	
	    	fourthStep.addParallel(fourthMotion);
	    	fourthStep.addParallel(fourthLiftControl);
	    	
	    	fifthLiftControl.addSequential(new DelayTime(1.5));
	    	fifthLiftControl.addSequential(new SetLiftScale());
	    	fifthLiftControl.addSequential(new DelayPathPosLeft(9.75));
	    	fifthLiftControl.addSequential(new SetIntakeState(IntakeControlState.OUT_SLOW));
	    	
	    	fifthStep.addParallel(new FollowPath("secondSwitchCubeToLeftScale"));
	    	fifthStep.addParallel(fifthLiftControl);
	    	
	    	addSequential(firstStep);
	    	addSequential(secondStep);
	    	addSequential(thirdStep);
	    	addSequential(fourthStep);
	    	addSequential(fifthStep);
    	}else if(gm.equals("LRL")){
    		firstLiftControl.addSequential(new SetIntakeState(IntakeControlState.INTAKE));
    		firstLiftControl.addSequential(new DelayTime(1.0));// TUNE THIS !!!!!
	    	firstLiftControl.addSequential(new SetLiftArmSetpoints(0.5, Constants.kScaleArmFlip));
	    	
	    	firstStep.addParallel(new FollowPath("leftToLeftSwitch"));
	    	firstStep.addParallel(firstLiftControl);
	    	
	    	addSequential(firstStep);
	    	addSequential(new SetIntakeState(IntakeControlState.OUT_FAST));
    	}else{
    		addSequential(new FollowPath("anyToCrossLineBackwards"));
    	}
    	
    	/*else if(gm.equals("RRR")||gm.equals("LRL")){
    		//addSequential(new FollowPath("anyToCrossLineBackwards"));
	    	firstLiftControl.addSequential(new SetIntakeState(IntakeControlState.INTAKE));
    		firstLiftControl.addSequential(new DelayPathPosLeft(33.0));// TUNE THIS !!!!!
	    	firstLiftControl.addSequential(new SetLiftScale());
	    	firstLiftControl.addSequential(new DelayPathPosLeft(37.5));// TUNE THIS !!!!!
	    	firstLiftControl.addSequential(new SetIntakeState(IntakeControlState.OUT_FAST));
    		
    		firstStep.addParallel(new FollowPath("leftToRightScale"));
    		firstStep.addParallel(firstLiftControl);
    		
    		//secondLiftControl.addSequential(new DelayTime(0.25));
	    	secondLiftControl.addSequential(new SetLiftArmSetpoints(Constants.kLiftSoftStopLow,Constants.kArmIntakeDeg));
	    	secondLiftControl.addSequential(new DelayTime(0.5));
	    	secondLiftControl.addSequential(new SetIntakeState(IntakeControlState.INTAKE));
	    	
	    	secondStep.addParallel(new FollowPath("rightScaleToSwitchCube"));
	    	secondStep.addParallel(secondLiftControl);

	    	thirdLiftControl.addSequential(new DelayTime(0.5));// TUNE THIS !!!!!
	    	thirdLiftControl.addSequential(new SetScaleState(ScaleState.LOW_NO_FLIP));
	    	thirdLiftControl.addSequential(new SetLiftScale());
	    	thirdLiftControl.addSequential(new DelayTime(2.5));// TUNE THIS !!!!!
	    	thirdLiftControl.addSequential(new SetIntakeState(IntakeControlState.OUT_SLOW));
	    	
	    	thirdStep.addParallel(new FollowPath("switchCubeToRightScale"));
	    	thirdStep.addParallel(thirdLiftControl);

	    	addSequential(firstStep);
	    	addSequential(secondStep);
	    	addSequential(thirdStep);
    	}*/
    }
}
