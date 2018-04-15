package org.usfirst.frc.team4587.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.*;

import edu.wpi.first.wpilibj.DigitalInput;
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
        MANUAL_IN,
        HOLD,
        INTAKE_OPEN,
    }

    // Control states
    private IntakeControlState mIntakeControlState = IntakeControlState.OFF;
    private IntakeControlState lastIntakeControlState = IntakeControlState.OFF;
    private IntakeControlState xIntakeControlState = IntakeControlState.OFF;
    
    private boolean cubeHasBeenLoaded, ultraHasCube, limitHasCube;

    // Hardware
    private final Spark intakeMotor0, intakeMotor1;
    private final Solenoid intakeClosePiston, intakeOpenPiston, intakeLED;
    private final DigitalInput limitSwitch, ultrasonic;
    int count;

    // Logging
    private DebugOutput mDebugOutput;
    private final ReflectingCSVWriter<DebugOutput> mCSVWriter;
    
    public IntakeControlState getIntakeState (){
    	synchronized (Intake.class) {
    		return xIntakeControlState;
    	}
    }
    public void setIntakeControlState(IntakeControlState state){
    	synchronized (Intake.class) {
    		xIntakeControlState = state;
    	}
    }
    int iCall = 0;
    private final Loop mLoop = new Loop() {

        @Override
        public void onStart(double timestamp) {
        	setIntakeControlState(IntakeControlState.OFF);
        	setMotorLevels (0.0);
        	startTime = System.nanoTime();
        	count=0;
        }

        @Override
        public void onLoop(double timestamp) {
        	if(!ultrasonic.get()){
        		count++;
        	}else{
        		count=0;
        		intakeLEDOff();
        	}
        	ultraHasCube = count>15;
        	if(ultraHasCube){
        		if(count<20){
        			intakeLEDOn();
        		}else if(count<25){
        			intakeLEDOff();
        		}else if(count<30){
        			intakeLEDOn();
        		}else if(count<35){
        			intakeLEDOff();
        		}else if(count<40){
        			intakeLEDOn();
        		}else if(count<45){
        			intakeLEDOff();
        		}
        	}
        	SmartDashboard.putBoolean("IRHasCube", ultraHasCube);
        	limitHasCube = limitSwitch.get();
        	synchronized (Intake.class){
        		mIntakeControlState = xIntakeControlState;
        	}
        	iCall++;
        	if(iCall % 1000 == 0){
        		System.out.println("onLoop " + iCall + " " + mIntakeControlState);
        	}
                switch (getIntakeState()) {
                case OFF:
                	setMotorLevels(0.0);
                	setIntakeClose(Constants.kIntakeCloseOn);
                	setIntakeOpen(Constants.kIntakeOpenOff);
                    break;
                case OUT_SLOW:
                	setMotorLevels(Constants.kIntakeOutSlow);
                	setIntakeClose(Constants.kIntakeCloseOn);
                	setIntakeOpen(Constants.kIntakeOpenOff);
                    break;
                case OUT_FAST:
                	setMotorLevels(Constants.kIntakeOutFast);
                	setIntakeClose(Constants.kIntakeCloseOn);
                	setIntakeOpen(Constants.kIntakeOpenOff);
                    break;
                case MANUAL_IN:
                	setMotorLevels(Constants.kIntakeIn);
                	setIntakeClose(Constants.kIntakeCloseOn);
                	setIntakeOpen(Constants.kIntakeOpenOn);
                	break;
                case HOLD:
                	setMotorLevels(Constants.kIntakeHold);
                	setIntakeClose(Constants.kIntakeCloseOn);
                	setIntakeOpen(Constants.kIntakeOpenOff);
                	break;
                case INTAKE_OPEN:
                	setMotorLevels(0.0);
                	setIntakeClose(Constants.kIntakeCloseOff);
                	setIntakeOpen(Constants.kIntakeOpenOn);
                	break;
                case INTAKE:
                	if(ultraHasCube){
                		setMotorLevels(Constants.kIntakeHold);
                    	setIntakeClose(Constants.kIntakeCloseOn);
                    	setIntakeOpen(Constants.kIntakeOpenOff);
                	}else{
                    	setMotorLevels(Constants.kIntakeIn);
                    	setIntakeClose(Constants.kIntakeCloseOn);
                    	setIntakeOpen(Constants.kIntakeOpenOn);
                	}
                	/*if(lastIntakeControlState!=mIntakeControlState){
                		//initialize
                		cubeHasBeenLoaded=false;
                	}
                	if(ultraHasCube && limitHasCube){
                		setMotorLevels(Constants.kIntakeHold);
                		cubeHasBeenLoaded=true;
                	}else{
                    	setMotorLevels(Constants.kIntakeIn);
                	}
                	if(ultraHasCube || limitHasCube){
                    	setIntakeClose(Constants.kIntakeCloseOn);
                    	setIntakeOpen(Constants.kIntakeOpenOff);
                	}else{
                		//cubeHasBeenLoaded
                    	setIntakeClose(Constants.kIntakeCloseOn);
                    	setIntakeOpen(Constants.kIntakeOpenOn);
                	}*/
                	
                	break;
                default:
                    System.out.println("Unexpected intake control state: " + mIntakeControlState);
                    break;
                }
            
        	logValues();
        	lastIntakeControlState = mIntakeControlState;
        }

        @Override
        public void onStop(double timestamp) {
            stop();
            mCSVWriter.flush();
        }
    };
    
    private void setMotorLevels(double x){
    	//double pdp0 = Robot.getPDP().getCurrent(RobotMap.INTAKE_0_SPARK_PDP);
    	//double pdp1 = Robot.getPDP().getCurrent(RobotMap.INTAKE_1_SPARK_PDP);
    	intakeMotor0.set(x);
    	//if(pdp0 >= Constants.kIntakeCurrentLimit || pdp1 >= Constants.kIntakeCurrentLimit){
    	//	x /= 2;
    	//}
    	intakeMotor1.set(-x);
    }
    private void setIntakeClose(boolean x){//MAKE PRIVATE!!
    	if(intakeClosePiston.get()!=x){
    		intakeClosePiston.set(x);
    	}
    }
    private void setIntakeOpen(boolean x){//MAKE PRIVATE!!
    	if(intakeOpenPiston.get()!=x){
    		intakeOpenPiston.set(x);
    	}
    }
    private void intakeLEDOn(){
    	if(intakeLED.get()!=true){
    		intakeLED.set(true);
    	}
    }
    private void intakeLEDOff(){
    	if(intakeLED.get()!=false){
    		intakeLED.set(false);
    	}
    }
	

	private Intake() {
        // Start all Talons in open loop mode.
		intakeMotor0 = new Spark(RobotMap.INTAKE_0_SPARK);
		intakeMotor1 = new Spark(RobotMap.INTAKE_1_SPARK);
		intakeClosePiston = new Solenoid(RobotMap.INTAKE_CLOSE);
		intakeOpenPiston = new Solenoid(RobotMap.INTAKE_OPEN);
		intakeLED = new Solenoid(RobotMap.INTAKE_LEDS);
		limitSwitch = new DigitalInput(RobotMap.INTAKE_LIMIT_SWITCH);
		ultrasonic = new DigitalInput(RobotMap.INTAKE_ULTRASONIC);
		
        mDebugOutput = new DebugOutput();
        mCSVWriter = new ReflectingCSVWriter<DebugOutput>("/home/lvuser/IntakeLog.csv",
                DebugOutput.class);
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
    	//SmartDashboard.putNumber("intake motor current", Robot.getPDP().getCurrent(RobotMap.INTAKE_0_SPARK_PDP));
    	SmartDashboard.putNumber("intake motor0 percent", intakeMotor0.get());
    	SmartDashboard.putNumber("intake motor1 percent", intakeMotor1.get());
    }
    
    public class DebugOutput{
    	public long sysTime;
    	public String intakeMode;
    	public double motor0Percent;
    	public double motor1Percent;
    	public double ultraInches;
    }
    
    public void logValues(){
    	synchronized(Intake.class){
	    	mDebugOutput.sysTime = System.nanoTime()-startTime;
	    	mDebugOutput.intakeMode = mIntakeControlState.name();
	    	mDebugOutput.motor0Percent = intakeMotor0.get();
	    	mDebugOutput.motor1Percent = intakeMotor1.get();
		    
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
@Override
protected void initDefaultCommand() {
	// TODO Auto-generated method stub
	
}       
}
