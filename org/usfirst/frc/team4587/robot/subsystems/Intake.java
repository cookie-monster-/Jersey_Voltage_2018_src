package org.usfirst.frc.team4587.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Ultrasonic;
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

public class Intake extends Subsystem {
	private long startTime;

    private static Intake mInstance = null;

    public static Intake getInstance() {
    	if ( mInstance == null ) {
    		synchronized ( Intake.class ) {
    			mInstance = new Intake();
    		}
    	}
    	return mInstance;
    }

    // The robot drivetrain's various states.
    public enum IntakeControlState {
        OFF, // open loop voltage control
        OUT_SLOW, // used for autonomous driving
        OUT_FAST,
        INTAKE,
        IN_SLOW,
    }

    // Control states
    private IntakeControlState mIntakeControlState = IntakeControlState.OFF;

    // Hardware
    private final Spark intakeMotor;
    private final Solenoid intakeGripPiston, intakeIntakePiston;
    private final Ultrasonic ultra;

    // Logging
    private DebugOutput mDebugOutput;
    private final ReflectingCSVWriter<DebugOutput> mCSVWriter;
    
    public void setOff (){
     	synchronized (Intake.class) {
     		mIntakeControlState = IntakeControlState.OFF;
     	}
    }
    public void setIntake (){
    	synchronized (Intake.class) {
    		mIntakeControlState = IntakeControlState.INTAKE;
    	}
    }
    public void setInSlow (){
    	synchronized (Intake.class) {
    		mIntakeControlState = IntakeControlState.IN_SLOW;
    	}
    }
    public void setOutSlow (){
    	synchronized (Intake.class) {
    		mIntakeControlState = IntakeControlState.OUT_SLOW;
    	}
    }
    public void setOutFast (){
    	synchronized (Intake.class) {
    		mIntakeControlState = IntakeControlState.OUT_FAST;
    	}
    }
    public IntakeControlState getIntakeState (){
    	synchronized (Intake.class) {
    		return mIntakeControlState;
    	}
    }
    public void setIntakeControlState(IntakeControlState state){
    	synchronized (Intake.class) {
    		mIntakeControlState = state;
    	}
    }
    /*
    public void runTest() {
    	System.out.println("in runTest");
    	synchronized (Intake.class) {
    		mIntakeControlState = IntakeControlState.TEST_MODE;
    	}
    }
    */
    int iCall = 0;
    private final Loop mLoop = new Loop() {

        @Override
        public void onStart(double timestamp) {
        	setOff();
        	setMotorLevels (0.0);
        	startTime = System.nanoTime();
        }

        @Override
        public void onLoop(double timestamp) {
        	iCall++;
        	if(iCall % 1000 == 0){
        		System.out.println("onLoop " + iCall + " " + mIntakeControlState);
        	}
                switch (getIntakeState()) {
                case OFF:
                	setMotorLevels(0.0);
                    break;
                case IN_SLOW:
                	if(Math.abs(Robot.getLift().getVel())>=0.00001 || Math.abs(Robot.getLift().getArmVel())>=0.00001){
                    	setMotorLevels(Constants.kIntakeInMedium);
                	}else{
                    	setMotorLevels(Constants.kIntakeInSlow);
                	}
                    break;
                case OUT_SLOW:
                	setMotorLevels(Constants.kIntakeOutSlow);
                    break;
                case OUT_FAST:
                	setMotorLevels(Constants.kIntakeOutFast);
                    break;
                case INTAKE:
                	setMotorLevels(Constants.kIntakeIn);
                	break;
                default:
                    System.out.println("Unexpected intake control state: " + mIntakeControlState);
                    break;
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
    	intakeMotor.set(-x);
    }
    public void setIntakeGrip(boolean x){//MAKE PRIVATE!!
    	intakeGripPiston.set(x);
    }
    public void setIntakeIntake(boolean x){//MAKE PRIVATE!!
    	intakeIntakePiston.set(x);
    }
    
	

	private Intake() {
        // Start all Talons in open loop mode.
		intakeMotor = new Spark(RobotMap.INTAKE_0_SPARK);
		intakeGripPiston = new Solenoid(RobotMap.INTAKE_GRIP);
		intakeIntakePiston = new Solenoid(RobotMap.INTAKE_INTAKE);
		ultra = new Ultrasonic(RobotMap.ULTRASONIC_PING,RobotMap.ULTRASONIC_ECHO);
		ultra.setAutomaticMode(true);
		
        mDebugOutput = new DebugOutput();
        mCSVWriter = new ReflectingCSVWriter<DebugOutput>("/home/lvuser/IntakeLog.csv",
                DebugOutput.class);
    }
	
	public double getUltraInches(){
		return ultra.getRangeInches();
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

    	//SmartDashboard.putNumber("intake motor current", Robot.getPDP().getCurrent(RobotMap.INTAKE_0_SPARK_PDP));
    	SmartDashboard.putNumber("intake motor percent", intakeMotor.get());
    	SmartDashboard.putNumber("ultra dist inches", getUltraInches());
    }
    
    public class DebugOutput{
    	public long sysTime;
    	public String intakeMode;
    	public double motorPercent;
    	public double ultraInches;
    }
    
    public void logValues(){
    	synchronized(Intake.class){
	    	mDebugOutput.sysTime = System.nanoTime()-startTime;
	    	mDebugOutput.intakeMode = mIntakeControlState.name();
	    	mDebugOutput.motorPercent = intakeMotor.get();
	    	mDebugOutput.ultraInches = ultra.getRangeInches();
		    
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
   

@Override
public void zeroSensors() {
	// TODO Auto-generated method stub
}       
}
