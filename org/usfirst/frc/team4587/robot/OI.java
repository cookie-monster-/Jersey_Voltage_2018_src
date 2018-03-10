/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team4587.robot;

import org.usfirst.frc.team4587.robot.commands.SetLiftHeight;
import org.usfirst.frc.team4587.robot.commands.ShiftClimbMode;
import org.usfirst.frc.team4587.robot.commands.runTest;
import org.usfirst.frc.team4587.robot.commands.startTeleopDrive;
import org.usfirst.frc.team4587.robot.commands.startTestPath;

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
	//JoyButton leftTrigger1, rightTrigger1;
	Joystick  stick2;
	Button	  buttonA2, buttonB2, buttonX2, buttonY2, leftBumper2, rightBumper2;
	//JoyButton leftTrigger2, rightTrigger2;

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
    	//leftTrigger1	= new JoyButton(stick1, JoyButton.JoyDir.DOWN, 2);
    	rightBumper1	= new JoystickButton(stick1, 6);
    	//rightTrigger1	= new JoyButton(stick1, JoyButton.JoyDir.DOWN, 3);
    	
    	stick2			= new Joystick(2);
    	buttonA2		= new JoystickButton(stick2, 1);
    	buttonB2		= new JoystickButton(stick2, 2);
    	buttonX2		= new JoystickButton(stick2, 3);
    	buttonY2		= new JoystickButton(stick2, 4);
    	leftBumper2 	= new JoystickButton(stick2, 5);
    	//leftTrigger2	= new JoyButton(stick2, JoyButton.JoyDir.DOWN, 2);
    	rightBumper2	= new JoystickButton(stick2, 6);
    	//rightTrigger2	= new JoyButton(stick2, JoyButton.JoyDir.DOWN, 3);
    	
    	System.out.println("OI start");
    	buttonA1.whenPressed(new startTeleopDrive());
    	buttonB1.whenPressed(new startTestPath());
    	buttonX1.whenPressed(new runTest());
    	//buttonX1.whenPressed(new SwitchSpark());
    	//buttonY1.whenPressed(new SwitchSparkDirection());

    	buttonA2.whenPressed(new SetLiftHeight(2.5));
    	buttonX2.whenPressed(new SetLiftHeight(-1.8));
    	buttonB2.whenPressed(new SetLiftHeight(0));
    	leftBumper2.whenPressed(new ShiftClimbMode(false));
    	rightBumper2.whenPressed(new ShiftClimbMode(true));
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
		double drive = -1 * stick2.getRawAxis(3);
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
}
