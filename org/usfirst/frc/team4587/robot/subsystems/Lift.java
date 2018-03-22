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
import org.usfirst.frc.team4587.robot.subsystems.Intake.IntakeControlState;
import org.usfirst.frc.team4587.robot.util.DriveSignal;

public class Lift extends Subsystem {
	
    private static Lift mInstance = null;

    public static Lift getInstance() {
    	if ( mInstance == null ) {
    		synchronized ( Lift.class ) {
    			mInstance = new Lift();
    		}
    	}
    	return mInstance;
    }
    
    int iCall = 0;
    private final Loop mLoop = new Loop() {

        @Override
        public void onStart(double timestamp) {
        	setOpenLoop();
        	setMotorLevels (0.0);
        	mStartTime = System.nanoTime();
        	mLastTime = mStartTime;
        	mPosLast = getPosFeet();
        	tBrakeOff = 0;
        	m_atSoftHigh = false;
        	m_atSoftLow = false;
        	if(Robot.getInTeleop() == false){
        		encoder.reset();
        	}
        	mEncoderFudge = 0;
        	mDistFudge = 0;
        }

        @Override
        public void onLoop(double timestamp) {
        	iCall++;
        	if(iCall % 1000 == 0){
        		System.out.println("onLoop " + iCall + " " + mLiftControlState);
        	}
            synchronized (Lift.class) {
            	liftShift.set(mIsClimbMode);
            	tCurrent = System.nanoTime();
            	dCurrent = getPosFeet();
            	vCurrent = getVelFPS();
                switch (mLiftControlState) {
                case OPEN_LOOP:
                	double liftDrive = OI.getInstance().getLiftDrive();
                	if(m_atSoftLow && liftDrive < 0){
                		liftDrive = 0;
                	}
                	if(m_atSoftHigh && liftDrive > 0){
                		liftDrive = 0;
                	}
                	if(liftDrive != 0.0){
                        if(mIsBrakeMode==Constants.kLiftBrakeOn){
                        	setBrakeOff();
                        }
                	}else{
                		setBrakeOn();
                	}

                	setMotorLevels(liftDrive);
                	SmartDashboard.putNumber("liftPosFeet: ", getPosFeet());
                    break;
                case PATH_FOLLOWING:
                case HOLD:
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
                case DEBUG:
                	setMotorLevels(OI.getInstance().getLiftDrive());
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
            mCSVWriter.flush();
        }
    };
    
    private void setMotorLevels(double x){
    	if(getLiftState() == LiftControlState.DEBUG){
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
    	if((mIsBrakeMode == Constants.kLiftBrakeOn) || ((tCurrent - tBrakeOff) < Constants.kLiftBrakeTimeToRelease)){
    		x=0;
    		SmartDashboard.putString("1st bool","1st bool"+(mIsBrakeMode == Constants.kLiftBrakeOn)+","+mIsBrakeMode+","+Constants.kLiftBrakeOn);
    		SmartDashboard.putString("2nd bool","2nd bool"+((tCurrent - tBrakeOff) < Constants.kLiftBrakeTimeToRelease)+","+tCurrent/(1000.0*1000.0*1000.0)+","+tBrakeOff/(1000.0*1000.0*1000.0)+","+Constants.kLiftBrakeTimeToRelease/(1000.0*1000.0*1000.0));
    	}
    	if(dCurrent>Constants.kLiftSoftStopHigh && x>0){
    		x=0;
        	//setBrakeOn();
    		m_atSoftHigh = true;
    	}else if(x<0){
    		m_atSoftHigh = false;
    	}
    	if(dCurrent<Constants.kLiftSoftStopLow && x<0){
    		x=0;
        	//setBrakeOn();
    		m_atSoftLow = true;
    	}else if(x>0){
    		m_atSoftLow = false;
    	}
    	if(Math.abs(x)>0){
    		m_liftIsMoving = true;
    		System.out.println("armDcurrent: "+Robot.getArm().getDCurrent()+" bool: "+(dCurrent < 0 && Math.abs(Robot.getArm().getDCurrent() - Constants.kArmSoftStopLifting) < Constants.kArmSoftStopLiftingTolerance));
    		if(dCurrent < 0 && Math.abs(Robot.getArm().getDCurrent() - Constants.kArmSoftStopLifting) > Constants.kArmSoftStopLiftingTolerance){
    			x=Constants.kLiftHoldLowPower;
    		}else if(dCurrent < Constants.kLiftSoftStopForArm && x < 0 && Robot.getArm().getDCurrent() > Constants.kArmSoftStopMiddle){
    			x=Constants.kLiftHoldHighPower;
    		}
    	}else{
    		m_liftIsMoving = false;
    	}
    	System.out.println("liftmotorset: "+x);
    	if(x != 0.0){
            if(mIsBrakeMode==Constants.kLiftBrakeOn){
            	setBrakeOff();
            }//shouldn't be here
    	}
    	x = -x;
    	liftMotor0.set(x);
    	liftMotor1.set(x);
    	liftMotor2.set(x);
    	liftMotor3.set(x);
    }
    
	PathFollower follower = null;
	
	private void doPathFollowing(){
		double error = m_setpoint - dCurrent;
		LiftControlState liftControlState;
		synchronized (Lift.class) {
			liftControlState = mLiftControlState;
		}
		if (liftControlState == LiftControlState.HOLD){
			/*double output = error * Constants.kLiftHoldKp + Math.min(error, lastError) * Constants.kLiftHoldKi - (error - lastError) * Constants.kLiftHoldKd;
			if(dCurrent > 0){
				output+=Constants.kLiftHoldHighPower;
			}else{
				output+=Constants.kLiftHoldLowPower;
			}
			setMotorLevels(output);*/
			setMotorLevels(0.0);
		}else{
            if(mIsBrakeMode==Constants.kLiftBrakeOn){
            	setBrakeOff();
            }//shouldn't need to turn off brake here
			if (error>1.0){
				setMotorLevels(1.0);
			}else if (error>0.025){//fix
				setMotorLevels(0.5);
			}else if(error<-1.0){
				setMotorLevels(-0.5);
			}else if(error<-0.025){//fix
				if(dCurrent>0){
					setMotorLevels(-0.2);
				}else{
					setMotorLevels(-0.3);
				}
			}else{
				//setOpenLoop();
				synchronized(Lift.class){
					mLiftControlState = LiftControlState.HOLD;
				}
				setBrakeOn();
			}
		}
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
    		mLiftControlState = LiftControlState.OPEN_LOOP;
    	}
    }
    public void setDebug (){
    	System.out.println("in setDebug");
       	synchronized (Lift.class) {
       		mLiftControlState = LiftControlState.DEBUG;
       	}
    }
       
    public void runTest() {
    	System.out.println("in runTest");
       	synchronized (Lift.class) {
       		mLiftControlState = LiftControlState.TEST_MODE;
       	}
    }
    
    public enum BrakeState {
    	ON,
    	OFF,
    	OPENING,
    }

    private BrakeState mBrakeState = BrakeState.ON;
    private BrakeState xBrakeState = BrakeState.ON;
    
    public void setBrakeState(BrakeState state){
    	synchronized (Lift.class){
    		xBrakeState = state;
    	}
    }
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
    		mDistFudge = getPosFeet();
    		mEncoderFudge = encoder.get();
    		xLiftControlState = LiftControlState.OPEN_LOOP;
        	xIsClimbMode = Constants.kLiftClimbOn;
    	}
    }
    public void stopClimbMode() {
    	System.out.println("in stopClimb");
    	synchronized (Lift.class) {
    		mDistFudge = getPosFeet();
    		mEncoderFudge = encoder.get();
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

	@Override
	public void zeroSensors() {
		synchronized (Lift.class){
			encoder.reset();
		}
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
    private final Spark liftMotor0, liftMotor1, liftMotor2, liftMotor3;
    private final Encoder encoder;
    private final Solenoid liftBrake, liftShift;
    
    private Lift() {
		liftMotor0 = new Spark(RobotMap.LIFT_0_SPARK);
		liftMotor1 = new Spark(RobotMap.LIFT_1_SPARK);
		liftMotor2 = new Spark(RobotMap.LIFT_2_SPARK);
		liftMotor3 = new Spark(RobotMap.LIFT_3_SPARK);

		encoder = new Encoder(RobotMap.LIFT_ENCODER_A,RobotMap.LIFT_ENCODER_B);
		liftBrake = new Solenoid(RobotMap.LIFT_BRAKE);
		liftShift = new Solenoid(RobotMap.LIFT_SHIFT);
		
        mDebugOutput = new DebugOutput();
        mCSVWriter = new ReflectingCSVWriter<DebugOutput>("/home/lvuser/LiftLog.csv",
                DebugOutput.class);
    }
    
    private long mCurrentTime, mLastTime, mStartTime, mBrakeOffTime;
	private boolean mAtSoftHigh, mAtSoftLow;
	private double mEncoderFudge, mDistFudge, mLiftDrive;
    
    private double getPosFeet(){
 	   if(mIsClimbMode == Constants.kLiftClimbOff){
 		   return mDistFudge + ((mPos - mEncoderFudge) * Constants.kLiftInchesPerTicHighGear / 12.0);
 	   }else{
 		   return mDistFudge + ((mPos - mEncoderFudge) * Constants.kLiftInchesPerTicLowGear / 12.0);
 	   }
    }
    
    private double getVelFPS(){
 	   return ((mPos - mPosLast) / (mCurrentTime - mLastTime)) * Math.pow(10, 9);
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
    	SmartDashboard.putNumber("Lift Height", getPos());
    	SmartDashboard.putString("Lift Mode", getLiftControlState().name());
    	SmartDashboard.putNumber("Lift Setpoint", getLiftSetpoint());
    	SmartDashboard.putString("Lift isScaleHigh", getScaleState().name());
    	//SmartDashboard.putNumber("lift motor0 current", Robot.getPDP().getCurrent(RobotMap.LIFT_0_SPARK_PDP));
    	//SmartDashboard.putNumber("lift motor1 current", Robot.getPDP().getCurrent(RobotMap.LIFT_1_SPARK_PDP));
    	//SmartDashboard.putNumber("lift motor2 current", Robot.getPDP().getCurrent(RobotMap.LIFT_2_SPARK_PDP));
    	//SmartDashboard.putNumber("lift motor3 current", Robot.getPDP().getCurrent(RobotMap.LIFT_3_SPARK_PDP));
    }

    // Logging
    private DebugOutput mDebugOutput;
    private final ReflectingCSVWriter<DebugOutput> mCSVWriter;
    
    public class DebugOutput{
    	public long sysTime;
    	public String liftMode;
    	public double encoder;
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
	    mDebugOutput.encoder = mPos;
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
