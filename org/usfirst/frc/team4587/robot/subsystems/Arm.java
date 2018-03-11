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
import org.usfirst.frc.team4587.robot.subsystems.Lift.LiftControlState;
import org.usfirst.frc.team4587.robot.util.DriveSignal;

public class Arm extends Subsystem {
	
	private long startTime;
	private double dexp0;
	private long tLast, tCurrent, tHitMaxCurrent;
	private double vCurrent, dCurrent, dLast, lastError, error;
	private double m_setpoint;
	private boolean m_atSoftHigh, m_atSoftLow, m_wasInDeadbandLast;
	public void setSetpoint(double degrees){
		synchronized ( Arm.class ) {
			m_setpoint = degrees;
		}
	}
	
	public double getDCurrent(){
		return dCurrent;
	}

    private static Arm mInstance = null;

    public static Arm getInstance() {
    	if ( mInstance == null ) {
    		synchronized ( Arm.class ) {
    			mInstance = new Arm();
    		}
    	}
    	return mInstance;
    }

    // The robot drivetrain's various states.
    public enum ArmControlState {
        OPEN_LOOP, // open loop voltage control
        PATH_FOLLOWING, // used for autonomous driving
        HOLD,
        DEBUG,
        TEST_MODE, // to run the testSubsystem() method once, then return to OPEN_LOOP
    }

    // Control states
    private ArmControlState mArmControlState = ArmControlState.OPEN_LOOP;

    // Hardware
    private final Spark armMotor;
    private final Encoder encoder;

    // Hardware states
    private boolean mNeverHitMaxCurrent;
    private boolean mStartingPath = false;

    // Logging
    private DebugOutput mDebugOutput;
    private final ReflectingCSVWriter<DebugOutput> mCSVWriter;
    
    public void startPath() {
    	System.out.println("in startPath");
    	synchronized (Arm.class) {
	    	mArmControlState = ArmControlState.PATH_FOLLOWING;
	    	mStartingPath = true ;
    	}
    }
     public void setOpenLoop (){
    	 System.out.println("in setOpenLoop");
     	synchronized (Arm.class) {
     		mArmControlState = ArmControlState.OPEN_LOOP;
     	}
     }
     public void setDebug (){
    	 System.out.println("in setDebug");
     	synchronized (Arm.class) {
     		mArmControlState = ArmControlState.DEBUG;
     	}
     }
     
     public ArmControlState getArmState (){
      	synchronized (Arm.class) {
      		return mArmControlState;
      	}
      }
    
    public void runTest() {
    	System.out.println("in runTest");
    	synchronized (Arm.class) {
    		mArmControlState = ArmControlState.TEST_MODE;
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
        	dLast = getPosDegrees();
        	mNeverHitMaxCurrent = true;
        	tHitMaxCurrent = 0;
        	m_atSoftHigh = false;
        	m_atSoftLow = false;
        	m_wasInDeadbandLast = false;
        	encoder.reset();
        }

        @Override
        public void onLoop(double timestamp) {
        	iCall++;
        	if(iCall % 1000 == 0){
        		System.out.println("onLoop " + iCall + " " + mArmControlState);
        	}
            synchronized (Arm.class) {
            	tCurrent = System.nanoTime();
            	dCurrent = getPosDegrees();
            	vCurrent = getVelDPS();
            	error = m_setpoint - dCurrent;
                switch (mArmControlState) {
                case OPEN_LOOP:
                	double armDrive = OI.getInstance().getArmDrive();
                	double safetySetting = 0;
                	if(m_atSoftLow && armDrive < 0){
                		armDrive = 0;
                	}
                	if(m_atSoftHigh && armDrive > 0){
                		armDrive = 0;
                	}

                	if(Robot.getLift().getLiftIsMoving() && Robot.getLift().getDCurrent() < 0){
                		//range -170 to -90
                		if(dCurrent < Constants.kArmSoftStopLifting){
                			setSetpoint(Constants.kArmSoftStopLifting);
                    		safetySetting = getPIDOutput();
                		}
                		if(dCurrent > Constants.kArmSoftStopMiddle){
                			setSetpoint(Constants.kArmSoftStopMiddle);
                			safetySetting = getPIDOutput();
                		}
                	}else if(Robot.getLift().getLiftIsMoving()){
                		// range -170 to 20
                		if(dCurrent < Constants.kArmSoftStopLifting){
                			setSetpoint(Constants.kArmSoftStopLifting);
                			safetySetting = getPIDOutput();
                		}
                		if(dCurrent > Constants.kArmSoftStopHigh){
                			setSetpoint(Constants.kArmSoftStopHigh);
                			safetySetting = getPIDOutput();
                		}
                	}else{
                		if(Robot.getLift().getDCurrent()>0.01){//dirty change to stop flip over at start of match
                			//range -180 to 20
                			if(dCurrent < Constants.kArmSoftStopLow){
                    			setSetpoint(Constants.kArmSoftStopLow);
                    			safetySetting = getPIDOutput();
                    		}
                    		if(dCurrent > Constants.kArmSoftStopHigh){
                    			setSetpoint(Constants.kArmSoftStopHigh);
                    			safetySetting = getPIDOutput();
                    		}
                		}else{
                			//range -180 to -90
                			if(dCurrent < Constants.kArmSoftStopLow){
                    			setSetpoint(Constants.kArmSoftStopLow);
                    			safetySetting = getPIDOutput();
                    		}
                    		if(dCurrent > Constants.kArmSoftStopMiddle){
                    			setSetpoint(Constants.kArmSoftStopMiddle);
                    			safetySetting = getPIDOutput();
                    		}
                		}
                	}
                	if(armDrive == 0){
                		armDrive = safetySetting;
                	}else if(armDrive * safetySetting < 0){
                		armDrive = safetySetting;
                	}else if(Math.abs(armDrive)<Math.abs(safetySetting)){
                		armDrive = safetySetting;
                	}
                	
                	if(armDrive == 0.0){
                		if(m_wasInDeadbandLast==false){
                			setSetpoint(dCurrent);
                		}
                		m_wasInDeadbandLast = true;
                		armDrive = getPIDOutput();
                	}else{
                		m_wasInDeadbandLast = false;
                	}

                	setMotorLevels(armDrive);
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
                	setMotorLevels(OI.getInstance().getArmDrive());
                	break;
                case TEST_MODE:
                	testSubsystem();
                	mArmControlState = ArmControlState.OPEN_LOOP;
                	break;
                default:
                    System.out.println("Unexpected arm control state: " + mArmControlState);
                    break;
                }
            }
        	logValues();
    		lastError = error;
        }

        @Override
        public void onStop(double timestamp) {
            stop();
            mCSVWriter.flush();
        }
    };
    
    private void setMotorLevels(double x){
    	if(getArmState() == ArmControlState.DEBUG){
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
    	/*if(Robot.getPDP().getCurrent(RobotMap.ARM_SPARK_PDP)>Constants.kArmMaxAmps){
    		x=0;
    		mNeverHitMaxCurrent = false;
    		tHitMaxCurrent = tCurrent;
    		setOpenLoop();

    	}*/
    	if(tHitMaxCurrent > 0 && (tCurrent - tHitMaxCurrent) < Constants.kArmTimeSinceHitMax){
    		x=0;
    	}
    	if(dCurrent>Constants.kArmSoftStopHigh && x>0){
    		x=0;
    		m_atSoftHigh = true;
    	}else if(x<0){
    		m_atSoftHigh = false;
    	}
    	if(dCurrent<Constants.kArmSoftStopLow && x<0){
    		x=0;
    		m_atSoftLow = true;
    	}else if(x>0){
    		m_atSoftLow = false;
    	}
    	
    	
    	x = -x;
    	armMotor.set(x);
    }
    
    private double getPIDOutput(){
    	double output = error * Constants.kArmHoldKp + Math.min(error, lastError) * Constants.kArmHoldKi - (error - lastError) * Constants.kArmHoldKd;
		output += Math.sin(dCurrent*Math.PI/180.0)*Constants.kArmHoldPower;
		return output;
    }
	
	private void doPathFollowing(){
		if (mStartingPath) {
    		mStartingPath = false;
    		dexp0 = dCurrent;
		}else{
			if(dCurrent > dexp0)	dexp0 = dCurrent;
		}
		ArmControlState armControlState;
		synchronized (Arm.class) {
			armControlState = mArmControlState;
		}
		if (armControlState == ArmControlState.HOLD){
			double output = error * Constants.kArmHoldKp + Math.min(error, lastError) * Constants.kArmHoldKi - (error - lastError) * Constants.kArmHoldKd;
			output += Math.sin(dCurrent*Math.PI/180.0)*Constants.kArmHoldPower;
			setMotorLevels(output);
		}else{
			if (error>10.0){
				setMotorLevels(Constants.kArmMaxMotorUp);
			}else if (error>5.0){
				setMotorLevels(Constants.kArmSlowMotorUp);
			}else if(error<-10.0){
				setMotorLevels(Constants.kArmSlowMotorDown);
			}else if(error<-5.0){
				setMotorLevels(Constants.kArmMaxMotorDown);
			}else{
				//setOpenLoop();
				synchronized(Arm.class){
					mArmControlState = ArmControlState.HOLD;
				}
			}
		}
		
		SmartDashboard.putNumber("dMax", dexp0);
	}

	private Arm() {
        // Start all Talons in open loop mode.
		armMotor = new Spark(RobotMap.ARM_SPARK);

		encoder = new Encoder(RobotMap.ARM_ENCODER_A,RobotMap.ARM_ENCODER_B);
		
        mDebugOutput = new DebugOutput();
        mCSVWriter = new ReflectingCSVWriter<DebugOutput>("/home/lvuser/ArmLog.csv",
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
    	SmartDashboard.putNumber("Arm Angle", getPosDegrees());
    	SmartDashboard.putNumber("Arm Encoder", encoder.get());
    	SmartDashboard.putBoolean("Arm NeverHitMaxCurrent", mNeverHitMaxCurrent);
    	SmartDashboard.putNumber("Arm Setpoint", m_setpoint);
    	SmartDashboard.putNumber("Arm Motor Percent", armMotor.get());
    	SmartDashboard.putBoolean("armSoftHigh", m_atSoftHigh);
    	SmartDashboard.putBoolean("armSoftLow", m_atSoftLow);
    	//SmartDashboard.putNumber("arm motor current", Robot.getPDP().getCurrent(RobotMap.ARM_SPARK_PDP));
    }
    
    public class DebugOutput{
    	public long sysTime;
    	public String armMode;
    	public double encoder;
    	public double motorPercent;
    	public double driveStick;
    	public boolean brakeMode;
    }
    
    public void logValues(){
    	synchronized(Arm.class){
	    	mDebugOutput.sysTime = System.nanoTime()-startTime;
	    	mDebugOutput.armMode = mArmControlState.name();
	    	mDebugOutput.encoder = encoder.get();
	    	mDebugOutput.motorPercent = armMotor.get();
		    mDebugOutput.driveStick = OI.getInstance().getArmDrive();
		    
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
   
   private double getPosDegrees(){
	   return encoder.get() * Constants.kArmDegreesPerTic;
   }
   
   private double getVelDPS(){
	   return ((dCurrent - dLast) / (tLast - tCurrent)) * Math.pow(10, 9);
   }

@Override
public void zeroSensors() {
	// TODO Auto-generated method stub
	encoder.reset();
}       
}
