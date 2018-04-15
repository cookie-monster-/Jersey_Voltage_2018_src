/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team4587.robot;

import java.io.File;
import java.io.FileWriter;

public class RobotMap {
	//CAN ID's
	public static final int DRIVE_RIGHT_TALON = 2;
	public static final int DRIVE_RIGHT_VICTOR_1 = 21;
	public static final int DRIVE_RIGHT_VICTOR_2 = 22;
	public static final int DRIVE_LEFT_TALON = 1;
	public static final int DRIVE_LEFT_VICTOR_1 = 11;
	public static final int DRIVE_LEFT_VICTOR_2 = 12;

	//PWM's
	public static final int LIFT_0_SPARK = 4;
	public static final int LIFT_1_SPARK = 5;
	public static final int LIFT_2_SPARK = 6;
	public static final int LIFT_3_SPARK = 7;
	public static final int ARM_SPARK = 3;
	public static final int INTAKE_0_SPARK = 0;
	public static final int INTAKE_1_SPARK = 2;
	public static final int TAILGATE_SPARK = 1;
	
	//Solenoid's
	public static final int LIFT_BRAKE = 3;
	public static final int LIFT_SHIFT = 2;
	public static final int INTAKE_CLOSE = 1;
	public static final int INTAKE_OPEN = 4;
	public static final int FLOOPERS = 5;
	public static final int INTAKE_LEDS = 7;
	
	//DIO's
	public static final int LIFT_ENCODER_A = 0;
	public static final int LIFT_ENCODER_B = 1;
	public static final int ARM_ENCODER_A = 2;
	public static final int ARM_ENCODER_B = 3;
	public static final int INTAKE_LIMIT_SWITCH = 6;
	public static final int INTAKE_ULTRASONIC = 4;

	//PDP slots
	public static final int DRIVE_RIGHT_TALON_PDP = 0;
	public static final int DRIVE_RIGHT_VICTOR_1_PDP = 1;
	public static final int DRIVE_RIGHT_VICTOR_2_PDP = 2;
	public static final int DRIVE_LEFT_TALON_PDP = 15;
	public static final int DRIVE_LEFT_VICTOR_1_PDP = 14;
	public static final int DRIVE_LEFT_VICTOR_2_PDP = 13;
	
	public static final int LIFT_0_SPARK_PDP = 7;
	public static final int LIFT_1_SPARK_PDP = 6;
	public static final int LIFT_2_SPARK_PDP = 5;
	public static final int LIFT_3_SPARK_PDP = 4;
	public static final int ARM_SPARK_PDP = 11;
	public static final int INTAKE_0_SPARK_PDP = 10;
	public static final int INTAKE_1_SPARK_PDP = 5;
	public static final int TAILGATE_SPARK_PDP = 4;
	
	public static void printPortList(){
		try{
			FileWriter w = new FileWriter(new File("/home/lvuser/PortList.csv") );
			String toWrite = "NAME,CAN/PWM,PDP_SLOT\n"+
					"DRIVE_RIGHT_TALON,"+DRIVE_RIGHT_TALON+","+DRIVE_RIGHT_TALON_PDP+"\n"+
					"DRIVE_RIGHT_VICTOR_1,"+DRIVE_RIGHT_VICTOR_1+","+DRIVE_RIGHT_VICTOR_1_PDP+"\n"+
					"DRIVE_RIGHT_VICTOR_2,"+DRIVE_RIGHT_VICTOR_2+","+DRIVE_RIGHT_VICTOR_2_PDP+"\n"+
					"DRIVE_LEFT_TALON,"+DRIVE_LEFT_TALON+","+DRIVE_LEFT_TALON_PDP+"\n"+
					"DRIVE_LEFT_VICTOR_1,"+DRIVE_LEFT_VICTOR_1+","+DRIVE_LEFT_VICTOR_1_PDP+"\n"+
					"DRIVE_LEFT_VICTOR_2,"+DRIVE_LEFT_VICTOR_2+","+DRIVE_LEFT_VICTOR_2_PDP+"\n"+
					"LIFT_0_SPARK,"+LIFT_0_SPARK+","+LIFT_0_SPARK_PDP+"\n"+
					"LIFT_1_SPARK,"+LIFT_1_SPARK+","+LIFT_1_SPARK_PDP+"\n"+
					"LIFT_2_SPARK,"+LIFT_2_SPARK+","+LIFT_2_SPARK_PDP+"\n"+
					"LIFT_3_SPARK,"+LIFT_3_SPARK+","+LIFT_3_SPARK_PDP+"\n"+
					"ARM_SPARK,"+ARM_SPARK+","+ARM_SPARK_PDP+"\n"+
					"INTAKE_0_SPARK,"+INTAKE_0_SPARK+","+INTAKE_0_SPARK_PDP+"\n"+
					"INTAKE_1_SPARK,"+INTAKE_1_SPARK+","+INTAKE_1_SPARK_PDP+"\n"+
					"TAILGATE_SPARK,"+TAILGATE_SPARK+","+TAILGATE_SPARK_PDP;
			w.write(toWrite);
			w.close();
		}catch(Exception e){}
	}
}
