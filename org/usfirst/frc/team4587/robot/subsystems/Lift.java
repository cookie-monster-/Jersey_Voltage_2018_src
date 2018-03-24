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
import org.usfirst.frc.team4587.robot.subsystems.Arm.ArmControlState;
import org.usfirst.frc.team4587.robot.subsystems.Intake.IntakeControlState;
import org.usfirst.frc.team4587.robot.util.DriveSignal;

public class Lift extends Subsystem {
	
    private static Lift mInstance = null;

    public static Lift getInstance() {
    	if ( mInstance == null ) {
    		synchronized ( Lift.class ) {
    			if ( mInstance == null ) {
    				mInstance = new Lift();
    			}
    		}
    	}
    	return mInstance;
    }
    
    int iCall = 0;
    private final Loop mLoop = new Loop() {

        @Override
        public void onStart(double timestamp) {
        	mPosLast = getPosFeet();
        	mBrakeOffTime = 0;
        	mAtSoftHigh = false;
        	mAtSoftLow = false;
        	if(mHaveCalledOnStart == false || DriverStation.getInstance().isFMSAttached() == false){
        		liftEncoder.reset();
        		armEncoder.reset();
            	mStartTime = System.nanoTime();
            	mLastTime = mStartTime;
            	mHaveCalledOnStart = true;
        	}
    		setDebug();
        	setLiftMotorLevels(0.0);
        	setOpenLoop();
        	setBrakeOn();
        	setScaleState(ScaleState.HIGH);
        	stopClimbMode();
        	mEncoderFudge = 0;
        	mDistFudge = 0;
        }

        @Override
        public void onLoop(double timestamp) {
        	iCall++;
        	if(iCall % 1000 == 0){
        		System.out.println("onLoop " + iCall + " " + mLiftControlState);
        	}
            mEncoder = liftEncoder.get();
            mArmEncoder = armEncoder.get();
        	mCurrentTime = System.nanoTime();
        	mPos = getPosFeet();
        	mVel = getVelFPS();
        	mArmPos = getArmPos();
        	mArmVel = getArmVel();
        	mArmError = mArmSetpoint - mArmPos;
        	if(Math.abs(mPos - mLiftSetpoint) < Constants.kLiftTolerance){//todo mArmSetpoint
        		mIsAtSetpoints = true;
        	}else{
        		mIsAtSetpoints = false;
        	}
            synchronized (Lift.class) {
            	if(mIsClimbMode != xIsClimbMode){
            		mDistFudge = getPosFeet();
            		mEncoderFudge = liftEncoder.get();
            		mIsClimbMode = xIsClimbMode;
                	liftShift.set(mIsClimbMode);
            	}
            	xPos = mPos;
            	xVel = mVel;
            	mLiftSetpoint = xLiftSetpoint;
            	mArmSetpoint = xArmSetpoint;
        		xIsAtSetpoints = mIsAtSetpoints;
        		mLiftControlState = xLiftControlState;
        		mScaleState = xScaleState;
            }
            if(mBrakeState == BrakeState.OPENING && (mBrakeOffTime - mCurrentTime) >= Constants.kLiftBrakeTimeToRelease){
            	mBrakeState = BrakeState.OFF;
            }
            switch (mLiftControlState) {
            case OPEN_LOOP:
                mLiftDrive = OI.getInstance().getLiftDrive();
                if(mAtSoftLow && mLiftDrive < 0){
                	mLiftDrive = 0;
                }
                if(mAtSoftHigh && mLiftDrive > 0){
                	mLiftDrive = 0;
                }
                if(mLiftDrive != 0.0){
                    setBrakeOff();
                }else{
                	setBrakeOn();
                }
               	setLiftMotorLevels(mLiftDrive);
                break;
            case SETPOINT:
                doPathFollowing();
                break;
            case DEBUG:
            	mLiftDrive = OI.getInstance().getLiftDrive();
                if(mLiftDrive != 0.0){
                    setBrakeOff();
                }else{
                	setBrakeOn();
                }
               	setLiftMotorLevels(mLiftDrive);
               	setArmMotorLevels(OI.getInstance().getArmDrive());
                break;
            case TEST_MODE:
                testSubsystem();
                mLiftControlState = LiftControlState.OPEN_LOOP;
                break;
            default:
                System.out.println("Unexpected lift control state: " + mLiftControlState);
                break;
            }
            synchronized(Lift.class){
            	xBrakeState = mBrakeState;
            }
            logValues();
            mLastArmError = mArmError;
        }
        

        @Override
        public void onStop(double timestamp) {
            onStart(timestamp);
            mCSVWriter.flush();
        }
    };
    
    private void setLiftMotorLevels(double x){
    	if(mLiftControlState == LiftControlState.DEBUG){
        	x = -x;
        	liftMotor0.set(x);
        	liftMotor1.set(x);
        	liftMotor2.set(x);
        	liftMotor3.set(x);
        	return;
    	}
    	if(x < Constants.kLiftMaxMotorDown){
    		x = Constants.kLiftMaxMotorDown;
    	}
    	if(x > Constants.kLiftMaxMotorUp){
    		x = Constants.kLiftMaxMotorUp;
    	}
    	if(mBrakeState != BrakeState.OFF){
    		x=0;
    	}
    	if(Math.abs(x)>0){
    		/*if(dCurrent < 0 && Math.abs(Robot.getArm().getDCurrent() - Constants.kArmSoftStopLifting) > Constants.kArmSoftStopLiftingTolerance){
    			x=Constants.kLiftHoldLowPower;
    		}else if(dCurrent < Constants.kLiftSoftStopForArm && x < 0 && Robot.getArm().getDCurrent() > Constants.kArmSoftStopMiddle){
    			x=Constants.kLiftHoldHighPower;
    		}*/
    	}
    	x = -x;
    	liftMotor0.set(x);
    	liftMotor1.set(x);
    	liftMotor2.set(x);
    	liftMotor3.set(x);
    }
    
    private void setArmMotorLevels(double x){
    	if(mLiftControlState == LiftControlState.DEBUG || mLiftControlState == LiftControlState.SETPOINT){
        	x = -x;
        	armMotor.set(x);
        	return;
    	}
    	if(x < Constants.kArmMaxMotorDown){
    		x = Constants.kArmMaxMotorDown;
    	}
    	if(x > Constants.kArmMaxMotorUp){
    		x = Constants.kArmMaxMotorUp;
    	}
    	x = -x;
    	armMotor.set(x);
    }
	
	private void doPathFollowing(){
		double error = mLiftSetpoint - mPos;
		double motorLevel = 0;
		if(Math.abs(error) < Constants.kLiftTolerance){
			motorLevel = 0;
		}else{
			if (error>1.0){
				motorLevel = 1.0;
			}else if (error>0.025){
				motorLevel = 0.5;
			}else if(error<-1.0){
				motorLevel = -0.5;
			}else if(error<-0.025){
				if(mPos>0){
					motorLevel = -0.2;
				}else{
					motorLevel = -0.3;
				}
			}else{
				motorLevel = 0;
			}
		}
		if(motorLevel == 0){
			setBrakeOn();
		}else{
			setBrakeOff();
		}
		setLiftMotorLevels(motorLevel);
		
	}

	

    

	
	// S H A R E D   A C C E S S
	// These member variables can be accessed by either thread, but only by calling the appopriate getter method.
	
	// The robot drivetrain's various states.
    public enum LiftControlState {
        OPEN_LOOP, // open loop voltage control
        SETPOINT,
        DEBUG,
        TEST_MODE, // to run the testSubsystem() method once, then return to OPEN_LOOP
    }

    // Control states
    private LiftControlState mLiftControlState = LiftControlState.OPEN_LOOP;
    private LiftControlState xLiftControlState = LiftControlState.OPEN_LOOP;
    
    public void setLiftControlState(LiftControlState state){
    	synchronized (Lift.class){
    		xLiftControlState = state;
    	}
    }
    public LiftControlState getLiftControlState(){
    	synchronized (Lift.class){
    		return xLiftControlState;
    	}
    }
    public void startSetpoint() {
    	System.out.println("in startSetpoint");
    	synchronized (Lift.class) {
        	if(getClimbMode() == Constants.kLiftClimbOff){ 
	    		xLiftControlState = LiftControlState.SETPOINT;
        	}
    	}
    }
    public void setOpenLoop (){
    	System.out.println("in setOpenLoop");
    	synchronized (Lift.class) {
    		xLiftControlState = LiftControlState.OPEN_LOOP;
    	}
    }
    public void setDebug (){
    	System.out.println("in setDebug");
       	synchronized (Lift.class) {
       		xLiftControlState = LiftControlState.DEBUG;
       	}
    }
       
    public void runTest() {
    	System.out.println("in runTest");
       	synchronized (Lift.class) {
       		xLiftControlState = LiftControlState.TEST_MODE;
       	}
    }
    
    public enum BrakeState {
    	ON,
    	OFF,
    	OPENING,
    }

    private BrakeState mBrakeState = BrakeState.ON;
    private BrakeState xBrakeState = BrakeState.ON;
   
    public BrakeState getBrakeState(){
    	synchronized (Lift.class){
    		return xBrakeState;
    	}
    }
    
    public enum ScaleState {
    	HIGH,
    	LOW,
    	//other
    }
    private ScaleState mScaleState = ScaleState.HIGH;
    private ScaleState xScaleState = ScaleState.HIGH;
    public void setScaleState(ScaleState state){
    	synchronized (Lift.class){
    		xScaleState = state;
    	}
    }
    public ScaleState getScaleState(){
    	synchronized (Lift.class){
    		return xScaleState;
    	}
    }
    
    private boolean mIsClimbMode = Constants.kLiftClimbOff;
    private boolean xIsClimbMode = Constants.kLiftClimbOff;
    public boolean getClimbMode(){
    	synchronized (Lift.class){
    		return xIsClimbMode;
    	}
    }
    
    public void startClimbMode() {
    	System.out.println("in startClimb");
    	synchronized (Lift.class) {   		
    		xLiftControlState = LiftControlState.OPEN_LOOP;
        	xIsClimbMode = Constants.kLiftClimbOn;
    	}
    }
    public void stopClimbMode() {
    	System.out.println("in stopClimb");
    	synchronized (Lift.class) {
    		xLiftControlState = LiftControlState.OPEN_LOOP;
        	xIsClimbMode = Constants.kLiftClimbOff;
    	}
    }
    
    private double mPos;
    private double mPosLast;
    private double xPos;
    public double getPos(){
    	synchronized (Lift.class){
    		return xPos;
    	}
    }

    private double mVel;
    private double xVel;
    public double getVel(){
    	synchronized (Lift.class){
    		return xVel;
    	}
    }
    
    private double mArmPos;
    private double mArmPosLast;
    private double xArmPos;
    public double getArmPos(){
    	synchronized (Lift.class){
    		return xArmPos;
    	}
    }

    private double mArmVel;
    private double xArmVel;
    public double getArmVel(){
    	synchronized (Lift.class){
    		return xArmVel;
    	}
    }

	@Override
	public void zeroSensors() { 
		
	}
    
    private double mLiftSetpoint;
    private double xLiftSetpoint;
    public void setLiftSetpoint(double setpoint){
    	synchronized (Lift.class){
    		xLiftSetpoint = setpoint;
    	}
    }
    public double getLiftSetpoint(){
    	synchronized (Lift.class){
    		return xLiftSetpoint;
    	}
    }
    
    private double mArmSetpoint;
    private double xArmSetpoint;
    public void setArmSetpoint(double setpoint){
    	synchronized (Lift.class){
    		xArmSetpoint = setpoint;
    	}
    }
    public double getArmSetpoint(){
    	synchronized (Lift.class){
    		return xArmSetpoint;
    	}
    }
    
    private boolean mIsAtSetpoints = false;
    private boolean xIsAtSetpoints = false;
    public boolean isAtSetpoint(){
    	return xIsAtSetpoints;
    }
    
	// S U B S Y S T E M   A C C E S S
	// These member variables can be accessed only by the subsystem

    // Hardware
    private final Spark liftMotor0, liftMotor1, liftMotor2, liftMotor3, armMotor;
    private final Encoder liftEncoder, armEncoder;
    private final Solenoid liftBrake, liftShift;
    
    private Lift() {
		liftMotor0 = new Spark(RobotMap.LIFT_0_SPARK);
		liftMotor1 = new Spark(RobotMap.LIFT_1_SPARK);
		liftMotor2 = new Spark(RobotMap.LIFT_2_SPARK);
		liftMotor3 = new Spark(RobotMap.LIFT_3_SPARK);
		armMotor = new Spark(RobotMap.ARM_SPARK);

		armEncoder = new Encoder(RobotMap.ARM_ENCODER_A,RobotMap.ARM_ENCODER_B);
		liftEncoder = new Encoder(RobotMap.LIFT_ENCODER_A,RobotMap.LIFT_ENCODER_B);
		liftBrake = new Solenoid(RobotMap.LIFT_BRAKE);
		liftShift = new Solenoid(RobotMap.LIFT_SHIFT);
		
        mDebugOutput = new DebugOutput();
        mCSVWriter = new ReflectingCSVWriter<DebugOutput>("/home/lvuser/LiftLog.csv", DebugOutput.class);
    }
    
    private long mCurrentTime, mLastTime, mStartTime, mBrakeOffTime;
	private boolean mAtSoftHigh, mAtSoftLow, mHaveCalledOnStart = false;
	private double mLiftDrive, mEncoder, mArmEncoder;
    private double mEncoderFudge=0, mDistFudge=0;
    private double mArmError, mLastArmError;
    
    private double getPosFeet(){
 	   if(mIsClimbMode == Constants.kLiftClimbOff){
 		   return mDistFudge + ((mEncoder - mEncoderFudge) * Constants.kLiftInchesPerTicHighGear / 12.0);
 	   }else{
 		   return mDistFudge + ((mEncoder - mEncoderFudge) * Constants.kLiftInchesPerTicLowGear / 12.0);
 	   }
    }
    
    private double getVelFPS(){
 	   return ((mPos - mPosLast) / (mCurrentTime - mLastTime)) * Math.pow(10, 9);
    }

    private double getPosDegrees(){
 	   return armEncoder.get() * Constants.kArmDegreesPerTic;
    }
    
    private double getVelDPS(){
 	   return ((mArmPos - mArmPosLast) / (mCurrentTime - mLastTime)) * Math.pow(10, 9);
    }
    
    private double getPIDOutput(){
    	double output = mArmError * Constants.kArmHoldKp + Math.min(mArmError, mLastArmError) * Constants.kArmHoldKi - (mArmError - mLastArmError) * Constants.kArmHoldKd;
		output += Math.sin(mArmPos*Math.PI/180.0)*Constants.kArmHoldPower;
		return output;
    }
    
    private void setBrakeOn(){
    	if(mBrakeState != BrakeState.ON){
    		mBrakeState = BrakeState.ON;
    		liftBrake.set(Constants.kLiftBrakeOn);
    	}
    }
    
    private void setBrakeOff(){
    	if(mBrakeState == BrakeState.ON){
    		mBrakeState = BrakeState.OPENING;
    		liftBrake.set(Constants.kLiftBrakeOff);
    		mBrakeOffTime = System.nanoTime();
    	}
    }
    
    @Override
    public void registerEnabledLoops(Looper in) {
        in.register(mLoop);
    }

    @Override
    public synchronized void stop() { //Called by robot thread.
        //setMotorLevels(0.0);
    	//The subsystem will set all needed values in its loop.onstop method.
    }
        
    @Override
    public void outputToSmartDashboard() {
    	SmartDashboard.putNumber("Lift Height", getPos());
    	SmartDashboard.putString("Lift Mode", getLiftControlState().name());
    	SmartDashboard.putNumber("Lift Setpoint", getLiftSetpoint());
    	SmartDashboard.putString("Lift isScaleHigh", getScaleState().name());	
    }

    // Logging
    private DebugOutput mDebugOutput;
    private final ReflectingCSVWriter<DebugOutput> mCSVWriter;
    
    public class DebugOutput{
    	public long sysTime;
    	public String liftMode;
    	public double liftEncoder;
    	public double posFeet;
    	public double armEncoder;
    	public double armPosDeg;
    	public double motorPercent;
    	public double driveStick;
    	public String brakeState;
    	public boolean brakeActual;
    	public double encoderFudge;
    	public double distFudge;
    }
    
    public void logValues(){
	    mDebugOutput.sysTime = System.nanoTime()-mStartTime;
	    mDebugOutput.liftMode = mLiftControlState.name();
	    mDebugOutput.liftEncoder = mEncoder;
	    mDebugOutput.posFeet = mPos;
	    mDebugOutput.armEncoder = mArmEncoder;
	    mDebugOutput.armPosDeg = mArmPos;
	    mDebugOutput.motorPercent = liftMotor0.get();
		mDebugOutput.driveStick = mLiftDrive;
		mDebugOutput.brakeState = mBrakeState.name();
		mDebugOutput.brakeActual = liftBrake.get();
		mDebugOutput.encoderFudge = mEncoderFudge;
		mDebugOutput.distFudge = mDistFudge;
		    
	    mCSVWriter.add(mDebugOutput);
	    SmartDashboard.putNumber("distFudge", mDistFudge);
	    SmartDashboard.putNumber("encoderFudge", mEncoderFudge);
    	SmartDashboard.putNumber("Lift Motor Percent", liftMotor0.get());
       	SmartDashboard.putNumber("liftPosFeet: ", mPos);
    	//SmartDashboard.putNumber("lift motor0 current", Robot.getPDP().getCurrent(RobotMap.LIFT_0_SPARK_PDP));
    	//SmartDashboard.putNumber("lift motor1 current", Robot.getPDP().getCurrent(RobotMap.LIFT_1_SPARK_PDP));
    	//SmartDashboard.putNumber("lift motor2 current", Robot.getPDP().getCurrent(RobotMap.LIFT_2_SPARK_PDP));
    	//SmartDashboard.putNumber("lift motor3 current", Robot.getPDP().getCurrent(RobotMap.LIFT_3_SPARK_PDP));
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
}
