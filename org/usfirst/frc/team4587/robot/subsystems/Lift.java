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
	
	private long startTime;
	private double dexp0;
	private long tLast, tCurrent, tHitMaxCurrent, tBrakeOff;
	private double vCurrent, dCurrent, dLast, offsetLast, lastError, encoderFudge, distFudge;
	private double m_setpoint;
	private boolean m_atSoftHigh, m_atSoftLow, m_liftIsMoving;
	
	public double getDCurrent(){
		return dCurrent;
	}
	public boolean getLiftIsMoving(){
		return m_liftIsMoving;
	}
	
	public void setSetpoint(double feet){
		synchronized ( Lift.class ) {
			m_setpoint = feet;
		}
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
        HOLD,
        DEBUG,
        TEST_MODE, // to run the testSubsystem() method once, then return to OPEN_LOOP
    }

    // Control states
    private LiftControlState mLiftControlState = LiftControlState.OPEN_LOOP;

    // Hardware
    private final Spark liftMotor0, liftMotor1, liftMotor2, liftMotor3;
    private final Encoder encoder;
    private final Solenoid liftBrake, liftShift;

    // Hardware states
    private boolean mIsBrakeMode = Constants.kLiftBrakeOn;
    private boolean mIsClimbMode = Constants.kLiftClimbOff;
    private boolean mNeverHitMaxCurrent;
    private boolean mStartingPath = false;
    private boolean mIsScaleHigh = true;
    public void setScaleHigh(boolean isScaleHigh){
    	mIsScaleHigh = isScaleHigh;
    }
    public boolean isScaleHigh(){
    	return mIsScaleHigh;
    }
    
    private void setBrakeOn(){
    	liftBrake.set(Constants.kLiftBrakeOn);
    	mIsBrakeMode = Constants.kLiftBrakeOn;
    }
    private void setBrakeOff(){
    	liftBrake.set(Constants.kLiftBrakeOff);
    	tBrakeOff = System.nanoTime();
    	mIsBrakeMode = Constants.kLiftBrakeOff;
    }

    // Logging
    private DebugOutput mDebugOutput;
    private final ReflectingCSVWriter<DebugOutput> mCSVWriter;
    
    public void startPath() {
    	System.out.println("in startPath");
    	synchronized (Lift.class) {
        	if(mIsClimbMode == Constants.kLiftClimbOff){
	    		mLiftControlState = LiftControlState.PATH_FOLLOWING;
	    		mStartingPath = true ;
        	}
    	}
    }
    public void startClimbMode() {
    	System.out.println("in startClimb");
    	synchronized (Lift.class) {
    		distFudge = getPosFeet();
    		encoderFudge = encoder.get();
    		mLiftControlState = LiftControlState.OPEN_LOOP;
    		liftShift.set(Constants.kLiftClimbOn);
        	mIsClimbMode = Constants.kLiftClimbOn;
    	}
    }
    public void stopClimbMode() {
    	System.out.println("in stopClimb");
    	synchronized (Lift.class) {
    		distFudge = getPosFeet();
    		encoderFudge = encoder.get();
    		mLiftControlState = LiftControlState.OPEN_LOOP;
    		liftShift.set(Constants.kLiftClimbOff);
        	mIsClimbMode = Constants.kLiftClimbOff;
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
    
     public LiftControlState getLiftState (){
     	synchronized (Lift.class) {
     		return mLiftControlState;
     	}
     }
    
    public void runTest() {
    	System.out.println("in runTest");
    	synchronized (Lift.class) {
    		mLiftControlState = LiftControlState.TEST_MODE;
    	}
    }
    
    int iCall = 0;
    private final Loop mLoop = new Loop() {

        @Override
        public void onStart(double timestamp) {
        	setOpenLoop();
        	setMotorLevels (0.0);
        	startTime = System.nanoTime();
        	tLast = startTime;
        	dLast = getPosFeet();
        	lastError = 0;
        	mNeverHitMaxCurrent = true;
        	tHitMaxCurrent = 0;
        	tBrakeOff = 0;
        	m_atSoftHigh = false;
        	m_atSoftLow = false;
        	if(Robot.getInTeleop() == false){
        		encoder.reset();
        	}
        	encoderFudge = 0;
        	distFudge = 0;
        }

        @Override
        public void onLoop(double timestamp) {
        	iCall++;
        	if(iCall % 1000 == 0){
        		System.out.println("onLoop " + iCall + " " + mLiftControlState);
        	}
            synchronized (Lift.class) {
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
    	/*if(Robot.getPDP().getCurrent(RobotMap.LIFT_0_SPARK_PDP)>Constants.kLiftMaxAmps ||
    			Robot.getPDP().getCurrent(RobotMap.LIFT_1_SPARK_PDP)>Constants.kLiftMaxAmps || 
    			Robot.getPDP().getCurrent(RobotMap.LIFT_2_SPARK_PDP)>Constants.kLiftMaxAmps ||
    			Robot.getPDP().getCurrent(RobotMap.LIFT_3_SPARK_PDP)>Constants.kLiftMaxAmps){
    		x=0;
    		mNeverHitMaxCurrent = false;
    		tHitMaxCurrent = tCurrent;
    		setOpenLoop();

    	}*/
    	if(tHitMaxCurrent > 0 && (tCurrent - tHitMaxCurrent) < Constants.kLiftTimeSinceHitMax){
    		//x=0;
    	}
    	if((mIsBrakeMode == Constants.kLiftBrakeOn)){// || ((tCurrent - tBrakeOff) < Constants.kLiftBrakeTimeToRelease)){
    		x=0;
    		SmartDashboard.putString("1st bool","1st bool"+(mIsBrakeMode == Constants.kLiftBrakeOn)+","+mIsBrakeMode+","+Constants.kLiftBrakeOn);
    		SmartDashboard.putString("2nd bool","2nd bool"+((tCurrent - tBrakeOff) < Constants.kLiftBrakeTimeToRelease)+","+tCurrent+","+tBrakeOff+","+Constants.kLiftBrakeTimeToRelease);
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
    		setBrakeOff();//shouldn't be here
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
		if (mStartingPath) {
			setBrakeOff();
    		mStartingPath = false;
    		dexp0 = dCurrent;
		}else{
			if(dCurrent > dexp0)	dexp0 = dCurrent;
		}
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
			setBrakeOff();//shouldn't need to turn off brake here
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
		lastError = error;
		
		SmartDashboard.putNumber("dMax", dexp0);
	}

	private Lift() {
        // Start all Talons in open loop mode.
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
        tLast = System.nanoTime();
        dLast = 0;
        encoder.reset();
        m_setpoint = 0;
		m_liftIsMoving = false;
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
    	SmartDashboard.putNumber("Lift Height", getPosFeet());
    	SmartDashboard.putNumber("distFudge", distFudge);
    	SmartDashboard.putNumber("encoderFudge", encoderFudge);
    	SmartDashboard.putBoolean("Lift NeverHitMaxCurrent", mNeverHitMaxCurrent);
    	SmartDashboard.putString("Lift Mode", mLiftControlState.name());
    	SmartDashboard.putNumber("Lift Setpoint", m_setpoint);
    	SmartDashboard.putNumber("Lift Motor Percent", liftMotor0.get());
    	SmartDashboard.putBoolean("Lift isScaleHigh", isScaleHigh());
    	//SmartDashboard.putNumber("lift motor0 current", Robot.getPDP().getCurrent(RobotMap.LIFT_0_SPARK_PDP));
    	//SmartDashboard.putNumber("lift motor1 current", Robot.getPDP().getCurrent(RobotMap.LIFT_1_SPARK_PDP));
    	//SmartDashboard.putNumber("lift motor2 current", Robot.getPDP().getCurrent(RobotMap.LIFT_2_SPARK_PDP));
    	//SmartDashboard.putNumber("lift motor3 current", Robot.getPDP().getCurrent(RobotMap.LIFT_3_SPARK_PDP));
    }
    
    public class DebugOutput{
    	public long sysTime;
    	public String liftMode;
    	public double encoder;
    	public double motorPercent;
    	public double driveStick;
    	public boolean brakeMode;
    	public boolean brakeActual;
    }
    
    public void logValues(){
    	synchronized(Lift.class){
	    	mDebugOutput.sysTime = System.nanoTime()-startTime;
	    	mDebugOutput.liftMode = mLiftControlState.name();
	    	mDebugOutput.encoder = encoder.get();
	    	mDebugOutput.motorPercent = liftMotor0.get();
		    mDebugOutput.driveStick = OI.getInstance().getLiftDrive();
		    mDebugOutput.brakeMode = mIsBrakeMode;
		    mDebugOutput.brakeActual = liftBrake.get();
		    
	    	mCSVWriter.add(mDebugOutput);
    	}
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
	   if(mIsClimbMode == Constants.kLiftClimbOff){
		   return distFudge + ((encoder.get() - encoderFudge) * Constants.kLiftInchesPerTicHighGear / 12.0);
	   }else{
		   return distFudge + ((encoder.get() - encoderFudge) * Constants.kLiftInchesPerTicLowGear / 12.0);
	   }
   }
   
   private double getVelFPS(){
	   return ((dCurrent - dLast) / (tLast - tCurrent)) * Math.pow(10, 9);
   }

@Override
public void zeroSensors() {
	// TODO Auto-generated method stub
	encoder.reset();
}       
}
