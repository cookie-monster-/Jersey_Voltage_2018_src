package org.usfirst.frc.team4587.robot.commands;

import org.usfirst.frc.team4587.robot.Constants;
import org.usfirst.frc.team4587.robot.subsystems.Intake.IntakeControlState;
import org.usfirst.frc.team4587.robot.subsystems.Lift.ScaleState;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class RightScaleAuto extends CommandGroup {

    public RightScaleAuto(String gm) {
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
    	if(gm.equals("LRL")||gm.equals("RRR")){
			filename1 = "rightToRightScale";
			filename2 = "rightScaleToSwitchCube";
			filename3 = "switchCubeToRightScale";
		

	    	firstLiftControl.addSequential(new DelayPathPosLeftOpp(14.0));// TUNE THIS !!!!!
	    	firstLiftControl.addSequential(new SetLiftScale());
	    	firstLiftControl.addSequential(new DelayPathPosLeftOpp(23.5));// TUNE THIS !!!!!
	    	firstLiftControl.addSequential(new SetIntakeState(IntakeControlState.OUT_FAST));
	    	firstLiftControl.addSequential(new DelayTime(0.85));
	    	firstLiftControl.addSequential(new SetLiftArmSetpoints(Constants.kLiftSoftStopLow,Constants.kArmIntakeDeg));
	
	    	firstStep.addParallel(new FollowPath(filename1));
	    	firstStep.addParallel(firstLiftControl);
	
	    	secondLiftControl.addSequential(new DelayTime(0.25));
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
	    	fifthLiftControl.addSequential(new DelayPathPosLeftOpp(9.75));
	    	fifthLiftControl.addSequential(new SetIntakeState(IntakeControlState.OUT_SLOW_AUTO));
	    	
	    	fifthStep.addParallel(new FollowPath("secondSwitchCubeToRightScale"));
	    	fifthStep.addParallel(fifthLiftControl);
	    	
	    	addSequential(firstStep);
	    	addSequential(secondStep);
	    	addSequential(thirdStep);
	    	addSequential(fourthStep);
	    	addSequential(fifthStep);
    	}else if(gm.equals("RLR")){
    		firstLiftControl.addSequential(new SetIntakeState(IntakeControlState.INTAKE));
    		firstLiftControl.addSequential(new DelayTime(3.0));
	    	firstLiftControl.addSequential(new SetLiftArmSetpoints(Constants.kLiftFlooperHeight, Constants.kArmFlooperDeg));
	    	
	    	firstMotion.addSequential(new FollowPath("rightToAlmostScale"));
	    	firstMotion.addSequential(new FollowPath("rightAlmostScaleToSwitchCube"));
	    	
	    	firstStep.addParallel(firstMotion);
	    	firstStep.addParallel(firstLiftControl);

	    	secondStep.addSequential(new DelayTime(0.5));
	    	secondStep.addSequential(new FollowPath("backupFoot"));
	    	secondStep.addSequential(new SetLiftArmSetpoints(Constants.kLiftSoftStopLow,Constants.kArmIntakeDeg));
	    	secondStep.addSequential(new SetIntakeState(IntakeControlState.INTAKE));
	    	secondStep.addSequential(new FollowPath("forwardFoot"));
	    	secondStep.addSequential(new DelayTime(0.5));
	    	secondStep.addSequential(new SetLiftArmSetpoints(Constants.kLiftFlooperHeight, Constants.kArmFlooperDeg));
	    	secondStep.addSequential(new FollowPath("forwardHalfFoot"));

	    	thirdStep.addSequential(new DelayTime(0.5));
	    	thirdStep.addSequential(new FollowPath("backupFoot"));
	    	thirdStep.addSequential(new SetLiftArmSetpoints(Constants.kLiftSoftStopLow,Constants.kArmIntakeDeg));
	    	thirdStep.addSequential(new SetIntakeState(IntakeControlState.INTAKE));
	    	thirdStep.addSequential(new FollowPath("thirdSwitchCubeRight"));
	    	
	    	addSequential(firstStep);
	    	addSequential(new SetIntakeState(IntakeControlState.OUT_FAST));
	    	addSequential(secondStep);
	    	addSequential(new SetIntakeState(IntakeControlState.OUT_FAST));
	    	addSequential(thirdStep);
    	}else{
    		//addSequential(new FollowPath("anyToCrossLineBackwards"));
	    	//addSequential(new SetIntakeState(IntakeControlState.INTAKE));
	    	//addSequential(new SetLiftArmSetpoints(Constants.kLiftSoftStopLow,Constants.kArmIntakeDeg));
    		addSequential(new FollowPath("rightToPlatform"));
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
