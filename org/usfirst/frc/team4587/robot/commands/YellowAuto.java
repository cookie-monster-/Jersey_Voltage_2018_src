package org.usfirst.frc.team4587.robot.commands;

import org.usfirst.frc.team4587.robot.Constants;
import org.usfirst.frc.team4587.robot.Robot;
import org.usfirst.frc.team4587.robot.subsystems.Intake.IntakeControlState;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class YellowAuto extends CommandGroup {

    public YellowAuto(String gm) {
    	//String gm = "LLL";//Robot.getGm();
    	CommandGroup firstLiftControl = new CommandGroup();
    	CommandGroup firstStep = new CommandGroup();
    	CommandGroup secondLiftControl = new CommandGroup();
    	CommandGroup secondMotion = new CommandGroup();
    	CommandGroup secondStep = new CommandGroup();
    	CommandGroup thirdLiftControl = new CommandGroup();
    	CommandGroup thirdStep = new CommandGroup();
    	
    	
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

    	//firstLiftControl.addSequential(new SetIntakeState(IntakeControlState.MANUAL_IN));
    	firstLiftControl.addSequential(new SetLiftArmSetpointsWait(1.0,Constants.kScaleArmFlip));
    	firstLiftControl.addSequential(new DelayTime(0.1));
    	firstLiftControl.addSequential(new SetLiftArmSetpointsWait(Constants.kLiftFlooperHeight,Constants.kArmFlooperDeg));

    	firstStep.addParallel(new FollowPath(filename1));
    	firstStep.addParallel(firstLiftControl);

    	secondLiftControl.addSequential(new DelayTime(0.1));
    	secondLiftControl.addSequential(new SetIntakeState(IntakeControlState.OUT_FAST));
    	secondLiftControl.addSequential(new SetIntakeState(IntakeControlState.OUT_FAST));
    	secondLiftControl.addSequential(new DelayTime(1.0));
    	secondLiftControl.addSequential(new SetLiftArmSetpoints(Constants.kLiftSoftStopLow,Constants.kArmIntakeDeg));
    	secondLiftControl.addSequential(new DelayTime(0.5));
    	secondLiftControl.addSequential(new SetIntakeState(IntakeControlState.INTAKE));

    	secondMotion.addSequential(new DelayTime(0.5));
    	secondMotion.addSequential(new FollowPath(filename2));
    	secondMotion.addSequential(new FollowPath(filename3));

    	secondStep.addParallel(secondLiftControl);
    	secondStep.addParallel(secondMotion);
    	
    	addSequential(firstStep);
    	addSequential(secondStep);
    	//addSequential(new FollowPath("backupToDriveStation"));
    	//addSequential(firstStep2);
    	//addSequential(secondStep2);

    	/*addSequential(new FollowPath(filename1));
    	addSequential(new FollowPath(filename2));
    	addSequential(new FollowPath(filename3));*/
    	addSequential(new FollowPath("pyramidToRightScale"));
    }
    protected void initialize(){
    	
    }
}
