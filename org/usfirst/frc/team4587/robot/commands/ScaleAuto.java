package org.usfirst.frc.team4587.robot.commands;

import org.usfirst.frc.team4587.robot.Constants;
import org.usfirst.frc.team4587.robot.subsystems.Intake.IntakeControlState;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class ScaleAuto extends CommandGroup {

    public ScaleAuto(String gm) {
    	//CommandGroup firstLiftControl = new CommandGroup();
    	CommandGroup firstStep = new CommandGroup();
    	//CommandGroup secondLiftControl = new CommandGroup();
    	CommandGroup secondMotion = new CommandGroup();
    	CommandGroup secondStep = new CommandGroup();
    	
    	String filename1="";
    	String filename2="";
    	String filename3 = "";
    	if(gm.equals("LRL")||gm.equals("RRR")){
			filename1 = "rightToRightScale";
			filename2 = "rightScaleToSwitchCube";
			filename3 = "switchCubeToRightScale";
		}else if(gm.equals("RLR")||gm.equals("LLL")){
			filename1 = "rightToLeftScale";
			filename2 = "leftScaleToSwitchCube";
			filename3 = "switchCubeToLeftScale";
		}
    	/*
    	firstLiftControl.addSequential(new DelayTime(0.5));
    	firstLiftControl.addSequential(new SetLiftArmSetpoints(Constants.kLiftFlooperHeight,Constants.kArmFlooperDeg));
    	firstLiftControl.addSequential(new DelayPathPos(10.0));// TUNE THIS !!!!!
    	firstLiftControl.addSequential(new SetIntakeState(IntakeControlState.OUT_FAST));
*/
    	firstStep.addParallel(new FollowPath(filename1));
  //  	firstStep.addParallel(firstLiftControl);
/*
    	secondLiftControl.addSequential(new DelayTime(0.5));
    	secondLiftControl.addSequential(new SetLiftArmSetpoints(Constants.kLiftSoftStopLow,Constants.kArmIntakeDeg));
    	secondLiftControl.addSequential(new DelayTime(0.5));
    	secondLiftControl.addSequential(new SetIntakeState(IntakeControlState.INTAKE));
*/
    	secondMotion.addSequential(new FollowPath(filename2));
    	secondMotion.addSequential(new FollowPath(filename3));
    	secondMotion.addSequential(new FollowPath("rightScaleToSecondSwitchCube"));
    	secondMotion.addSequential(new FollowPath("secondSwitchCubeToRightScale"));

  //  	secondStep.addParallel(secondLiftControl);
    	secondStep.addParallel(secondMotion);
    	
    	addSequential(firstStep);
    	addSequential(secondStep);
    }
}
