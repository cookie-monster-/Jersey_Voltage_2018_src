package org.usfirst.frc.team4587.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motion.MotionProfileStatus;
import com.ctre.phoenix.motion.SetValueMotionProfile;
import com.ctre.phoenix.motion.TrajectoryPoint;
import com.ctre.phoenix.motion.TrajectoryPoint.TrajectoryDuration;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import org.usfirst.frc.team4587.robot.util.Gyro;
import org.usfirst.frc.team4587.robot.util.ReflectingCSVWriter;

import java.io.File;
import java.io.FileWriter;

import org.usfirst.frc.team4587.robot.Constants;
import org.usfirst.frc.team4587.robot.OI;
import org.usfirst.frc.team4587.robot.Robot;
import org.usfirst.frc.team4587.robot.RobotMap;
//import com.team254.frc2017.RobotState;
//import com.team254.frc2017.ShooterAimingParameters;
import org.usfirst.frc.team4587.robot.loops.Loop;
import org.usfirst.frc.team4587.robot.loops.Looper;
import org.usfirst.frc.team4587.robot.paths.PathFollower;
import org.usfirst.frc.team4587.robot.util.DriveSignal;

public class Lift extends Subsystem {
	
	private long startTime;
	private double dexp0;
	private double dexp1;
	private double dexp2;
	private long tLast, tCurrent;
	private double vCurrent, dCurrent, dLast, offsetLast;
	private double m_setpoint;
	public void setSetpoint(double feet){
		m_setpoint = feet;
	}

    private static Lift mInstance = null;

    public static Lift getInstance() {
    	if ( mInstance == null ) {
    		synchronized ( Lift.class ) {
    			mInstance = new Lift();
    		}
    	}
    	return mInstance;
    }

    // The robot drivetrain's various states.
    public enum LiftControlState {
        OPEN_LOOP, // open loop voltage control
        PATH_FOLLOWING, // used for autonomous driving
        TEST_MODE, // to run the testSubsystem() method once, then return to OPEN_LOOP
    }

    public int getEnc(){
    	return encoder.get();
    }

    // Control states
    private LiftControlState mLiftControlState = LiftControlState.OPEN_LOOP;

    // Hardware
    private final Spark liftMotor0, liftMotor1, liftMotor2, liftMotor3;
    private final Encoder encoder;

    // Hardware states
    private boolean mIsBrakeMode;
    private boolean mStartingPath = false;

    // Logging
    private DebugOutput mDebugOutput;
    private final ReflectingCSVWriter<DebugOutput> mCSVWriter;
    
    public void startPath() {
    	System.out.println("in startPath");
    	synchronized (Lift.this) {
    		mLiftControlState = LiftControlState.PATH_FOLLOWING;
    		mStartingPath = true ;
    	}
    }
     public void setOpenLoop (){
    	 System.out.println("in setOpenLoop");
     	synchronized (Lift.this) {
     		mLiftControlState = LiftControlState.OPEN_LOOP;
     		setMotorLevels(0.0);
     	}
     }
    
    public void runTest() {
    	System.out.println("in runTest");
    	synchronized (Lift.this) {
    		mLiftControlState = LiftControlState.TEST_MODE;
    	}
    }
    
    int iCall = 0;
    private final Loop mLoop = new Loop() {

        @Override
        public void onStart(double timestamp) {
            synchronized (Lift.this) {
            	setOpenLoop();
            	startTime = System.nanoTime();
            	tLast = startTime;
            	dLast = getPosFeet();
            }
        }

        @Override
        public void onLoop(double timestamp) {
        	iCall++;
        	if(iCall % 1000 == 0){
        		System.out.println("onLoop " + iCall + " " + mLiftControlState);
        	}
            synchronized (Lift.this) {
            	tCurrent = System.nanoTime();
            	dCurrent = getPosFeet();
            	vCurrent = getVelFPS();
                switch (mLiftControlState) {
                case OPEN_LOOP:
                	setMotorLevels(OI.getInstance().getLiftDrive());
                	SmartDashboard.putNumber("liftPosFeet: ", getPosFeet());
                    break;
                case PATH_FOLLOWING:
                    //mLeftMaster.setInverted(true);
                    //mRightMaster.setInverted(true);
                    //_rightSlave1.setInverted(true);
                    //_rightSlave2.setInverted(true);
                	doPathFollowing();
                   // if (mPathFollower != null) {
                   //     updatePathFollower(timestamp);
                   //     mCSVWriter.add(mPathFollower.getDebug());
                //    }
                    break;
                case TEST_MODE:
                	testSubsystem();
                	mLiftControlState = LiftControlState.OPEN_LOOP;
                	break;
                default:
                    System.out.println("Unexpected lift control state: " + mLiftControlState);
                    break;
                }
            }
        	logValues();
        }

        @Override
        public void onStop(double timestamp) {
            stop();
            //mCSVWriter.flush();
        }
    };
    
    private void setMotorLevels(double x){
    	//System.out.println("left: "+left+" right: "+right);
    	if(x < -1){
    		x =-1;
    	}
    	if(x > 1){
    		x = 1;
    	}
    	liftMotor0.set(x);
    	liftMotor1.set(x);
    	liftMotor2.set(x);
    	liftMotor3.set(x);
    }
    
	PathFollower follower = null;
	
	private void doPathFollowing(){
		if ((m_setpoint - dCurrent)>0.75){
			setMotorLevels(1.0);
		}else if ((m_setpoint - dCurrent)>0.025){
			setMotorLevels(0.5);
		}else{
			setOpenLoop();
		}
		if (mStartingPath) {
    		mStartingPath = false;
    		dexp0 = dCurrent;
		}else{
			if(dCurrent > dexp0)	dexp0 = dCurrent;
		}
		SmartDashboard.putNumber("dMax", dexp0);
		/*double d0 = dCurrent;
		double v0 = vCurrent;
		double v1, a1, d1, d2, dexp;
    	if (mStartingPath) {
    		mStartingPath = false;
    		dexp = d0;
    	}else{
    		long deltaT = tCurrent - tLast;
    		if(deltaT <= Constants.kStepSizeSeconds){
    			//use dexp0 and dexp1
    			dexp = dexp0 + (dexp1 - dexp0)*(deltaT/Constants.kStepSizeSeconds);
    		}else if(deltaT <= Constants.kStepSizeSeconds*2){
    			//use dexp1 and dexp2
    			dexp = dexp1 + (dexp2 - dexp1)*((deltaT-Constants.kStepSizeSeconds)/Constants.kStepSizeSeconds);
    		}else{
    			dexp = dexp2;
    		}
    	}
    	double offset = m_setpoint - d0;
    	if(Math.signum(offset) * Math.signum(v0) < 0){
    		//opposite velocity
    		a1 = -1 * Math.signum(v0)*Constants.kLiftAMax;
    		v1 = v0 + (a1 * Constants.kStepSizeSeconds);
    	}else{
	    	
	    	double tStop = Math.abs(v0) / Constants.kLiftAMax;
	    	double dStop = (Math.abs(v0)*tStop) - (0.5*Constants.kLiftAMax*tStop*tStop);
	    	if(dStop >= Math.abs(offset)){
	    		//stop NOW!
	    		a1 = -1 * Math.signum(v0)*Constants.kLiftAMax;
	    		v1 = v0 + (a1 * Constants.kStepSizeSeconds);
	    	}else if(v0 >= Constants.kLiftVMax){
	    		//at max vel already
	    		a1 = 0;
	    		v1 = v0;
	    	}else{
	    		//need to accelerate
	    		a1 = Math.signum(offset)*Constants.kLiftAMax;
	    		v1 = v0 + (a1 * Constants.kStepSizeSeconds);
		    	double tStop1 = Math.abs(v1) / Constants.kLiftAMax;
		    	double dStop1 = (Math.abs(v1)*tStop1) - (0.5*Constants.kLiftAMax*tStop1*tStop1);
		    	if (dStop1 > Math.abs(offset)){
		    		a1 = 0;
		    		v1 = v0;
		    	}
	    	}
    	}
		d1 = d0 + v0 * Constants.kStepSizeSeconds + 0.5 * a1 * Constants.kStepSizeSeconds * Constants.kStepSizeSeconds;
		d2 = d1 + v1 * Constants.kStepSizeSeconds + 0.5 * a1 * Constants.kStepSizeSeconds * Constants.kStepSizeSeconds;
		if(Math.abs(offset)< 0.1){
			a1=0;
			v1=0;
		}
		double motorLevel = Constants.kLiftKa * (a1+32) + Constants.kLiftKv * v1 + Constants.kLiftKp * (dexp - d0);
    	setMotorLevels(motorLevel);
    	System.out.println(motorLevel);
    	SmartDashboard.putString("nums", "Ka: "+Constants.kLiftKa * a1 + " Kv: "+Constants.kLiftKv * v1 + " Kp: " +Constants.kLiftKp * (dexp - d0));
    	dexp0 = d0;
    	dexp1 = d1;
    	dexp2 = d2;
    	SmartDashboard.putNumber("d0", d0);
    	SmartDashboard.putNumber("d1", d1);
    	SmartDashboard.putNumber("d2", d2);
    	SmartDashboard.putNumber("v0", v0);
    	SmartDashboard.putNumber("v1", v1);
    	SmartDashboard.putNumber("a1", a1);
    	SmartDashboard.putNumber("offset", offset);
    	SmartDashboard.putNumber("dexp", dexp);
    	offsetLast = offset;*/
	}

	private Lift() {
        // Start all Talons in open loop mode.
		liftMotor0 = new Spark(RobotMap.LIFT_0_SPARK);
		liftMotor1 = new Spark(RobotMap.LIFT_1_SPARK);
		liftMotor2 = new Spark(RobotMap.LIFT_2_SPARK);
		liftMotor3 = new Spark(RobotMap.LIFT_3_SPARK);

		encoder = new Encoder(RobotMap.LIFT_ENCODER_A,RobotMap.LIFT_ENCODER_B);
        mDebugOutput = new DebugOutput();
        mCSVWriter = new ReflectingCSVWriter<DebugOutput>("/home/lvuser/DriveLog.csv",
                DebugOutput.class);
        tLast = System.nanoTime();
        dLast = 0;
        encoder.reset();
        m_setpoint = 0;
    }
    @Override
    public void registerEnabledLoops(Looper in) {
        in.register(mLoop);
    }

    @Override
    public synchronized void stop() {
        setMotorLevels(0.0);
    }

    @Override
    public void outputToSmartDashboard() {
        /*SmartDashboard.putNumber("left percent output", mLeftMaster.getMotorOutputPercent());
        SmartDashboard.putNumber("right percent output", mRightMaster.getMotorOutputPercent());
        SmartDashboard.putNumber("left position (rotations)", mLeftMaster.getSelectedSensorPosition(0));///4096);
        SmartDashboard.putNumber("right position (rotations)", mRightMaster.getSelectedSensorPosition(0));///4096);
        SmartDashboard.putNumber("gyro pos", Gyro.getYaw());*/
    }
    
    public class DebugOutput{
    	public long sysTime;
    	public String liftMode;
    	public double encoder;
    	public double motorPercent;
    	public double driveStick;
    	public double leftPathPos;
    	public double leftPathVel;
    	public double leftPathAcc;
    	public double pathHdg;
    	public int pathStep0;
    	public int pathStep1;
    }
    
    public void logValues(){
    	mDebugOutput.sysTime = System.nanoTime()-startTime;
    	mDebugOutput.liftMode = mLiftControlState.name();
    	mDebugOutput.encoder = encoder.get();
    	mDebugOutput.motorPercent = 0;//mLeftMaster.getMotorOutputPercent();
    	if(mLiftControlState == LiftControlState.OPEN_LOOP){
	    	mDebugOutput.driveStick = OI.getInstance().getLiftDrive();
	    	mDebugOutput.leftPathPos = 0;
	    	mDebugOutput.leftPathVel = 0;
	    	mDebugOutput.leftPathAcc = 0;
	    	mDebugOutput.pathHdg = 0;
	    	mDebugOutput.pathStep0 = 0;
	    	mDebugOutput.pathStep1 = 0;
    	}else if(follower!=null){
    		mDebugOutput.driveStick = 0;
	    	mDebugOutput.leftPathPos = follower.getXLeft();
	    	mDebugOutput.leftPathVel = follower.getVLeft();
	    	mDebugOutput.leftPathAcc = follower.getALeft();
	    	mDebugOutput.pathHdg = follower.getDesiredAngle();
	    	mDebugOutput.pathStep0 = follower.getStep0();
	    	mDebugOutput.pathStep1 = follower.getStep1();
    	}
    	mCSVWriter.add(mDebugOutput);
    }

       @Override
    public void writeToLog() {
        mCSVWriter.write();
    }

   public boolean testSubsystem(){

	   boolean all_ok = false;
	   try{
		   FileWriter w = new FileWriter(new File("/home/lvuser/testLog.csv"));
	   all_ok = true;

	   // Let the onLoop() method enable safety mode again...
	   w.close();
	   }catch(Exception e){}
	   return all_ok;
   }
   
   private double getPosFeet(){
	   return encoder.get() * Constants.kLiftInchesPerTic / 12.0;
   }
   
   private double getVelFPS(){
	   return ((dCurrent - dLast) / (tLast - tCurrent)) * Math.pow(10, 9);
   }

@Override
public void zeroSensors() {
	// TODO Auto-generated method stub
	
}       
}
