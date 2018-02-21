package org.usfirst.frc.team4587.robot.subsystems;

import org.usfirst.frc.team4587.robot.RobotMap;
import org.usfirst.frc.team4587.robot.loops.Looper;

import edu.wpi.first.wpilibj.Spark;

public class TestTheSparks {
	Spark intake0;
	Spark intake1;
	Spark arm;
	Spark tailgate;
	Spark lift0;
	Spark lift1;
	Spark lift2;
	Spark lift3;
	int count;
	
	public TestTheSparks(){
		intake0 = new Spark(RobotMap.INTAKE_0_SPARK);
		intake1 = new Spark(RobotMap.INTAKE_1_SPARK);
		arm = new Spark(RobotMap.ARM_SPARK);
		tailgate = new Spark(RobotMap.TAILGATE_SPARK);
		lift0 = new Spark(RobotMap.LIFT_0_SPARK);
		lift1 = new Spark(RobotMap.LIFT_1_SPARK);
		lift2 = new Spark(RobotMap.LIFT_2_SPARK);
		lift3 = new Spark(RobotMap.LIFT_3_SPARK);
		count = 0;
	}
	
	private Spark getCurrentSpark(){
		switch(count%8){
		case 0: return intake0;
		case 1: return intake1;
		case 2: return arm;
		case 3: return tailgate;
		case 4: return lift0;
		case 5: return lift1;
		case 6: return lift2;
		case 7: return lift3;
		}
		return null;
	}
	
	public void goToNextSpark(){
		getCurrentSpark().set(0.0);
		count++;
		getCurrentSpark().set(0.0);
	}
	
	public void setCurrentSparkSpeed(){
		double speed = getCurrentSpark().get();
		speed+=0.5;
		if(speed>1){
			speed = -1;
		}
		getCurrentSpark().set(speed);
	}
	
    public void writeToLog() {
    };
}