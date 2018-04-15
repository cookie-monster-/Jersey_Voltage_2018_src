/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team4587.robot;

import org.usfirst.frc.team4587.robot.commands.ClimbMode;
import org.usfirst.frc.team4587.robot.commands.IntakeAuto;
import org.usfirst.frc.team4587.robot.commands.SetDebugMode;
import org.usfirst.frc.team4587.robot.commands.SetIntakeState;
import org.usfirst.frc.team4587.robot.commands.SetLiftArmSetpoints;
import org.usfirst.frc.team4587.robot.commands.SetLiftScale;
import org.usfirst.frc.team4587.robot.commands.SetManualMode;
import org.usfirst.frc.team4587.robot.commands.SetScaleState;
import org.usfirst.frc.team4587.robot.commands.StopClimbMode;
import org.usfirst.frc.team4587.robot.subsystems.Intake.IntakeControlState;
import org.usfirst.frc.team4587.robot.subsystems.Lift.ScaleState;
import org.usfirst.frc.team4587.robot.util.JoyButton;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
//import utility.JoyButton;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
	private static OI mInstance = null;
	private Joystick stick1;
	Button	  buttonA1, buttonB1, buttonX1, buttonY1, leftBumper1, rightBumper1;
	JoyButton leftTrigger1, rightTrigger1;
	Joystick  stick2;
	Button	  buttonA2, buttonB2, buttonX2, buttonY2, leftBumper2, rightBumper2;
	JoyButton leftTrigger2, rightTrigger2;
	Joystick  driverStation;
	Button    toggleSwitch0, toggleSwitch1, toggleSwitch2, toggleSwitch3, toggleSwitch4;
	Button	  count0Button1, count0Button2, count1Button1, count1Button2, count2Button1, count2Button2, count3Button1, count3Button2;

	// Return the singleton OI object, creating it if necessary.
	// Creating the object is synchronized, just in case two threads end up calling simultaneously.
	public static OI getInstance()
	{
		if(mInstance == null) {
			synchronized ( OI.class ) {
				mInstance = new OI();
			}
		}
		return mInstance;
	}
	
	public OI()
	{
		//stick1 = new Joystick(1);
		stick1			= new Joystick(1);
    	buttonA1		= new JoystickButton(stick1, 1);
    	buttonB1		= new JoystickButton(stick1, 2);
    	buttonX1		= new JoystickButton(stick1, 3);
    	buttonY1		= new JoystickButton(stick1, 4);
    	leftBumper1 	= new JoystickButton(stick1, 5);
    	leftTrigger1	= new JoyButton(stick1, JoyButton.JoyDir.DOWN, 2);
    	rightBumper1	= new JoystickButton(stick1, 6);
    	rightTrigger1	= new JoyButton(stick1, JoyButton.JoyDir.DOWN, 3);
    	
    	stick2			= new Joystick(2);
    	buttonA2		= new JoystickButton(stick2, 1);
    	buttonB2		= new JoystickButton(stick2, 2);
    	buttonX2		= new JoystickButton(stick2, 3);
    	buttonY2		= new JoystickButton(stick2, 4);
    	leftBumper2 	= new JoystickButton(stick2, 5);
    	leftTrigger2	= new JoyButton(stick2, JoyButton.JoyDir.DOWN, 2);
    	rightBumper2	= new JoystickButton(stick2, 6);
    	rightTrigger2	= new JoyButton(stick2, JoyButton.JoyDir.DOWN, 3);
    	
    	driverStation   = new Joystick(0);
    	toggleSwitch0   = new JoystickButton(driverStation, 1);
    	toggleSwitch1   = new JoystickButton(driverStation, 2);
    	// turn auto off button 14
    	count0Button1 	= new JoystickButton(driverStation, 11);
    	count0Button2 	= new JoystickButton(driverStation, 10);
    	count1Button1 	= new JoystickButton(driverStation, 13);
    	count1Button2 	= new JoystickButton(driverStation, 12);
    	count2Button1 	= new JoystickButton(driverStation, 15);
    	count2Button2 	= new JoystickButton(driverStation, 16);
    	count3Button1 	= new JoystickButton(driverStation, 9);
    	count3Button2 	= new JoystickButton(driverStation, 8);
    	
    	System.out.println("OI start");
    	//buttonA1.whenPressed(new startTeleopDrive());
    	//buttonB1.whenPressed(new startTestPath());
    	//buttonX1.whenPressed(new runTest());
    	//buttonX1.whenPressed(new SwitchSpark());
    	//buttonY1.whenPressed(new SwitchSparkDirection());

    	/*buttonA2.whenPressed(new SetLiftHeight(3.1));
    	buttonB2.whenPressed(new SetLiftHeight(-1.4));
    	buttonX2.whenPressed(new SetLiftHeight(0.0));
    	*/
    	//buttonB2.whenPressed(new SetLiftHeight(0));
    	//buttonA2.whenPressed(new ToggleDebugMode());
    	
    	
    	//buttonY2.whenPressed(new SetLiftAndArmPosition(3.1,0,0));
    	//leftBumper2.whenPressed(new ShiftClimbMode(false));
    	//rightBumper2.whenPressed(new ShiftClimbMode(true));
    	
    	//buttonA2.whenPressed(new IntakeGripOn());
    	//buttonB2.whenPressed(new IntakeGripOff());
    	
    	/*
    	leftTrigger1.whileHeld(new IntakeOutSlow());
    	leftTrigger1.whenReleased(new IntakeStop());
    	rightTrigger1.whileHeld(new IntakeOutFast());
    	rightTrigger1.whenReleased(new IntakeStop());
    	leftBumper1.whenPressed(new SetScaleLow());
    	rightBumper1.whenPressed(new SetScaleHigh());


    	buttonA2.whenPressed(new SetLiftAndArmPosition(-1.45,-170,-180));
    	//buttonX2.whenPressed(new SetLiftAndArmPosition(3.1,-170,0));
    	//buttonY2.whenPressed(new SetLiftAndArmPosition(3.1,-170,-180));
    	buttonX2.whenPressed(new SetLiftScaleFlip());
    	buttonY2.whenPressed(new SetLiftScale());
    	buttonB2.whenPressed(new SetLiftAndArmPosition(0.5,-170,-180));

    	leftBumper2.whenPressed(new SetManualMode());
    	rightBumper2.whenPressed(new SetDebugMode());
    	leftTrigger2.whileHeld(new IntakeIn());
    	leftTrigger2.whenReleased(new IntakeSlowAndGrip());
    	*/
    	//buttonA1.whenPressed(new IntakeAuto());
    	/*buttonA1.whenPressed(new SetLiftArmSetpoints(-1.45,-180));
    	buttonB1.whenPressed(new SetLiftArmSetpoints(0,-180));
    	buttonX1.whenPressed(new SetLiftArmSetpoints(0.5,0.0));
    	buttonY1.whenPressed(new SetLiftArmSetpoints(2.0,0));
    	leftTrigger1.whileHeld(new IntakeOutSlow());
    	leftTrigger1.whenReleased(new IntakeStop());
    	rightTrigger1.whileHeld(new IntakeOutFast());
    	rightTrigger1.whenReleased(new IntakeStop());
    	leftBumper1.whenPressed(new IntakeAuto());*/
    	//buttonA1.whenPressed(new IntakeAuto());
    	
    	buttonA1.whenPressed(new SetScaleState(ScaleState.LOW_FLIP));
    	buttonY1.whenPressed(new SetScaleState(ScaleState.HIGH_FLIP));
    	buttonB1.whenPressed(new SetScaleState(ScaleState.LOW_NO_FLIP));
    	buttonX1.whenPressed(new SetScaleState(ScaleState.HIGH_NO_FLIP));
    	leftTrigger1.whileHeld(new SetIntakeState(IntakeControlState.OUT_SLOW));
    	leftTrigger1.whenReleased(new SetIntakeState(IntakeControlState.OFF));
    	rightTrigger1.whileHeld(new SetIntakeState(IntakeControlState.OUT_FAST));
    	rightTrigger1.whenReleased(new SetIntakeState(IntakeControlState.OFF));
    	rightBumper1.whenPressed(new SetIntakeState(IntakeControlState.INTAKE_OPEN));
    	rightBumper1.whenReleased(new SetIntakeState(IntakeControlState.OFF));

    	buttonA2.whenPressed(new SetLiftArmSetpoints(Constants.kLiftSoftStopLow,Constants.kArmIntakeDeg));
    	buttonB2.whenPressed(new SetLiftArmSetpoints(Constants.kLiftFlooperHeight,Constants.kArmFlooperDeg));
    	buttonX2.whenPressed(new SetLiftArmSetpoints(0.5, Constants.kScaleArmFlip));
    	buttonY2.whenPressed(new SetLiftScale());
    	leftBumper2.whenPressed(new SetDebugMode());
    	rightBumper2.whenPressed(new IntakeAuto());
    	rightTrigger2.whileHeld(new SetIntakeState(IntakeControlState.MANUAL_IN));
    	rightTrigger2.whenReleased(new SetIntakeState(IntakeControlState.HOLD));

    	toggleSwitch0.whileHeld(new ClimbMode());
    	toggleSwitch0.whenReleased(new StopClimbMode());
    	toggleSwitch1.whenPressed(new SetLiftArmSetpoints(2.83,17.0));
	}

	// Get the value of the "drive" stick.
	public double getDrive()
	{
		return -1 * stick1.getRawAxis(1);
	}

	// Get the value of the "turn" stick.
	public double getTurn()
	{
		return stick1.getRawAxis(4);
	}
	
	public double getLiftDrive()
	{
		double drive = -1 * stick2.getRawAxis(1);
		if(Math.abs(drive) < Constants.kLiftJoystickDeadband){
			return 0.0;
		}else if(drive < 0.5 && drive > 0){
			return Constants.kLiftSlowMotorUp;
		}else if(drive > 0.5){
			return Constants.kLiftMaxMotorUp;
		}else if(drive > -0.5 && drive < 0){
			return Constants.kLiftSlowMotorDown;
		}else{
			return Constants.kLiftMaxMotorDown;
		}
	}
	public double getArmDrive()
	{
		double drive = -1 * stick2.getRawAxis(5);
		if(Math.abs(drive) < Constants.kArmJoystickDeadband){
			return 0.0;
		}else if(drive < 0.5 && drive > 0){
			return Constants.kArmSlowMotorUp;
		}else if(drive > 0.5){
			return Constants.kArmMaxMotorUp;
		}else if(drive > -0.5 && drive < 0){
			return Constants.kArmSlowMotorDown;
		}else{
			return Constants.kArmMaxMotorDown;
		}
	}
	public int getCount0(){
		return (count0Button1.get() ? 1 : 0) + (count0Button2.get() ? 2 : 0);
	}
	public int getCount1(){
		return (count1Button1.get() ? 1 : 0) + (count1Button2.get() ? 2 : 0);
	}
	public int getCount2(){
		return (count2Button1.get() ? 1 : 0) + (count2Button2.get() ? 2 : 0);
	}
	public int getCount3(){
		return (count3Button1.get() ? 1 : 0) + (count3Button2.get() ? 2 : 0);
	}
}
