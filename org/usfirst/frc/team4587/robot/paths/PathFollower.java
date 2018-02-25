package org.usfirst.frc.team4587.robot.paths;

import java.io.BufferedReader;
import java.io.FileWriter;
import java.io.IOException;

import org.usfirst.frc.team4587.robot.Constants;
import org.usfirst.frc.team4587.robot.Robot;
import org.usfirst.frc.team4587.robot.subsystems.Drive;
import org.usfirst.frc.team4587.robot.util.Gyro;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Trajectory;

public class PathFollower {

	BufferedReader m_bufferedReader;
	private PathReader mPath;
	boolean quit;
	int m_startEncoderLeft;
	int m_startEncoderRight;
	double m_startAngle;
	double m_startTime;
	double Ka = Constants.kPathFollowKa;
	double Kv = Constants.kPathFollowKv;
	double Kp = Constants.kPathFollowKp;
	double Kg = Constants.kPathFollowKg;
	FileWriter m_logWriter;
	String m_namePath;
	Trajectory leftPath;
	Trajectory rightPath;

	double aLeft;
	public double getALeft(){
		return aLeft;
	}
	double vLeft;
	public double getVLeft(){
		return vLeft;
	}
	double xLeft;
	public double getXLeft(){
		return xLeft;
	}
	double aRight;
	public double getARight(){
		return aRight;
	}
	double vRight;
	public double getVRight(){
		return vRight;
	}
	double xRight;
	public double getXRight(){
		return xRight;
	}
	int step0;
	public int getStep0(){
		return step0;
	}
	int step1;
	public int getStep1(){
		return step1;
	}
	double desiredAngle;
	public double getDesiredAngle(){
		return desiredAngle;
	}
	
	double m_finalPositionRight;
	public double getFinalPositionRight(){
		return m_finalPositionRight;
	}
	double m_finalPositionLeft;
	public double getFinalPositionLeft(){
		return m_finalPositionLeft;
	}
	double m_leftMotorSetting;
	public double getLeftMotorSetting(){
		return m_leftMotorSetting;
	}
	double m_rightMotorSetting;
	public double getRightMotorSetting(){
		return m_rightMotorSetting;
	}
	
	private void setMotorLevels(double l, double r){
		m_leftMotorSetting = l;
		m_rightMotorSetting = r;
	}
    public PathFollower(String namePath) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	m_namePath = namePath;
    	mPath = new PathReader(m_namePath);
    	leftPath = mPath.left;
    	rightPath = mPath.right;
    }

    // Called just before this Command runs the first time
    public void initialize() {
    	quit = false;

		try {
			m_logWriter = new FileWriter("/home/lvuser/" + m_namePath +"Log.csv", false);
			m_logWriter.write("aLeft,vLeft,xLeft,aRight,vRight,xRight,desiredAngle,currentAngle,realLeftEncoder,realRightEncoder,leftMotorLevel,rightMotorLevel,System.nanoTime()" + "\n");
		} catch ( IOException e ) {
			System.out.println(e);
			m_logWriter = null;
		}
    	m_startEncoderLeft = Robot.getDrive().getLeftEnc();
    	m_startEncoderRight = Robot.getDrive().getRightEnc();
    	m_startTime = System.nanoTime();

    	m_finalPositionLeft = mPath.left.get(mPath.left.length()-1).position * 12 / Constants.kInchesPerTic;
    	m_finalPositionRight = mPath.right.get(mPath.right.length()-1).position * 12 / Constants.kInchesPerTic;
    }

    // Called repeatedly when this Command is scheduled to run
    public void execute() {
    	double time = System.nanoTime();
    	double dt = (time - m_startTime) / 1000000;
    	step0 = (int)(dt / 10);
    	step1 = step0 + 1;
    	double offset = dt - 10 * step0;
    	
    	
    	if(step1 >= leftPath.length())
    	{
    		quit = true;
    		step0 = step1 = leftPath.length()-1;
    	}
	    		Trajectory.Segment left0;
	        	Trajectory.Segment right0;
	    		Trajectory.Segment left1;
	        	Trajectory.Segment right1;
	        	
            	left0 = leftPath.get(step0);
            	left1 = leftPath.get(step1);
            	right0 = rightPath.get(step0);
            	right1 = rightPath.get(step1);

	        	xLeft = (left0.position + ((offset / 10) * (left1.position - left0.position))) * 12 / 0.0046;
	        	xRight = (right0.position + ((offset / 10) * (right1.position - right0.position))) * 12 / 0.0046;
	        	if(step0 == step1){
	            	aLeft = 0;
	            	vLeft = 0;
	            	aRight = 0;
		        	vRight = 0;
		        	Ka = 0;
		        	Kv = 0;
		        	Kp = Constants.kPathHoldKp;
		        	Kg = Constants.kPathHoldKg;
	        	}else{
	        		aLeft = (left0.acceleration + ((offset / 10) * (left1.acceleration - left0.acceleration))) * 12 / 0.0046 / 1000 * 10 / 1000 * 10;
	        		vLeft = (left0.velocity + ((offset / 10) * (left1.velocity - left0.velocity))) * 12 / 0.0046 / 1000 * 10;//convert ft/sec to ticks/10ms
	        		aRight = (right0.acceleration + ((offset / 10) * (right1.acceleration - right0.acceleration))) * 12 / 0.049 / 1000 * 10 / 1000 * 10;
	        		vRight = (right0.velocity + ((offset / 10) * (right1.velocity - right0.velocity))) * 12 / 0.0046 / 1000 * 10;
	        		Ka = Constants.kPathFollowKa;
	        		Kv = Constants.kPathFollowKv;
	        		Kp = Constants.kPathFollowKp;
	        		Kg = Constants.kPathFollowKg;
	        	}
        		desiredAngle = -right0.heading * 180 / Math.PI; //* -1;
        		double currentAngle = Gyro.getYaw();
        		int realLeftEncoder = Robot.getDrive().getLeftEnc();
        		int realRightEncoder = Robot.getDrive().getRightEnc();
        		desiredAngle += m_startAngle;
        		while(desiredAngle > 180)
        		{
        			desiredAngle -= 360;
        		}
        		while(desiredAngle < -180)
        		{
        			desiredAngle += 360;
        		}
        		xLeft += m_startEncoderLeft;
        		xRight += m_startEncoderRight;
        		
        		double leftMotorLevel = Ka * aLeft + Kv * vLeft - Kp * (realLeftEncoder - xLeft) - Kg * (currentAngle - desiredAngle);
        		double rightMotorLevel = Ka * aRight + Kv * vRight - Kp * (realRightEncoder - xRight) + Kg * (currentAngle - desiredAngle);
        		
        		setMotorLevels(leftMotorLevel, -rightMotorLevel);
        		SmartDashboard.putNumber("left motor set to: ", leftMotorLevel);
        		SmartDashboard.putNumber("right motor set to: ", -rightMotorLevel);
            	
        		if(m_logWriter != null)
        		{
        			try{
        				m_logWriter.write(aLeft + "," + vLeft + "," + xLeft + "," + aRight + "," + vRight + "," + xRight + "," + desiredAngle + "," + currentAngle + "," + realLeftEncoder + "," + realRightEncoder + "," + leftMotorLevel + "," + rightMotorLevel + "," + time + "\n");
        			}catch(Exception e){
        				
        			}
        		}
        	
    }

    // Make this return true when this Command no longer needs to run execute()
    public boolean isFinished() {
        return quit;
    }

    // Called once after isFinished returns true
    protected void end() {
    	try
    	{
    		m_logWriter.close();
    	}
    	catch(Exception e)
    	{
    		
    	}
    }

}