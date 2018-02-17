package org.usfirst.frc.team4587.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
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

public class Drive extends Subsystem {

    private static Drive mInstance = null;
    private DifferentialDrive _drive;

    public static Drive getInstance() {
    	if(mInstance == null)
    	{
    		mInstance = new Drive();
    	}
        return mInstance;
    }

    // The robot drivetrain's various states.
    public enum DriveControlState {
        OPEN_LOOP, // open loop voltage control
        PATH_FOLLOWING, // used for autonomous driving
    }
    
    public int getLeftEnc(){
    	return mLeftMaster.getSelectedSensorPosition(0);
    }
    public int getRightEnc(){
    	return mRightMaster.getSelectedSensorPosition(0);
    }

    // Control states
    private DriveControlState mDriveControlState = DriveControlState.OPEN_LOOP;

    // Hardware
    private final WPI_TalonSRX mLeftMaster, mRightMaster;
    private final WPI_VictorSPX _leftSlave1, _leftSlave2, _rightSlave1, _rightSlave2;
    private final Gyro mNavXBoard;

    // Hardware states
    private boolean mIsBrakeMode;
    private boolean mStartingPath = false;

    // Logging
    
    //private final ReflectingCSVWriter<PathFollower.DebugOutput> mCSVWriter;
    public void startPath() {
    	System.out.println("in startPath");
    	synchronized (Drive.this) {
    		mDriveControlState = DriveControlState.PATH_FOLLOWING;
    		mStartingPath = true ;
    	}
    }
    
    int iCall = 0;
    private final Loop mLoop = new Loop() {

        @Override
        public void onStart(double timestamp) {
            synchronized (Drive.this) {
            	
                setOpenLoop(DriveSignal.NEUTRAL);
                setBrakeMode(false);
                mNavXBoard.reset();
            }
        }

        @Override
        public void onLoop(double timestamp) {
        	iCall++;
        	if(iCall % 1000 == 0){
        		System.out.println("onLoop " + iCall + " " + mDriveControlState + " " + mLeftMaster.getControlMode());
        	}
            synchronized (Drive.this) {
                switch (mDriveControlState) {
                case OPEN_LOOP:
                	_drive.arcadeDrive(OI.getInstance().getDrive(), OI.getInstance().getTurn());
                    //mLeftMaster.setInverted(false);
                    mRightMaster.setInverted(false);
                    _rightSlave1.setInverted(false);
                    _rightSlave2.setInverted(false);
                    _drive.setSafetyEnabled(true);
                    return;
                case PATH_FOLLOWING:
                    //mLeftMaster.setInverted(true);
                    //mRightMaster.setInverted(true);
                    //_rightSlave1.setInverted(true);
                    //_rightSlave2.setInverted(true);
                    _drive.setSafetyEnabled(false);
                	doPathFollowing();
                   // if (mPathFollower != null) {
                   //     updatePathFollower(timestamp);
                   //     mCSVWriter.add(mPathFollower.getDebug());
                //    }
                    return;
                default:
                    System.out.println("Unexpected drive control state: " + mDriveControlState);
                    break;
                }
            }
        }

        @Override
        public void onStop(double timestamp) {
            stop();
            //mCSVWriter.flush();
        }
    };
    
    private void setMotorLevels(double left, double right){
    	System.out.println("left: "+left+" right: "+right);
    	if(left < -1){
    		left =-1;
    	}
    	if(left > 1){
    		left = 1;
    	}
    	if(right < -1){
    		right =-1;
    	}
    	if(right > 1){
    		right = 1;
    	}
    	mLeftMaster.set(ControlMode.PercentOutput, left);
    	mRightMaster.set(ControlMode.PercentOutput, right);
    }
    
    private TrajectoryDuration GetTrajectoryDuration(int durationMs)
	{	 
		/* create return value */
		TrajectoryDuration retval = TrajectoryDuration.Trajectory_Duration_0ms;
		/* convert duration to supported type */
		retval = retval.valueOf(durationMs);
		/* check that it is valid */
		if (retval.value != durationMs) {
			DriverStation.reportError("Trajectory Duration not supported - use configMotionProfileTrajectoryPeriod instead", false);		
		}
		/* pass to caller */
		return retval;
	}
    
	class PeriodicRunnable implements java.lang.Runnable {
	    public void run() { 
	    	mLeftMaster.processMotionProfileBuffer();
	    	mRightMaster.processMotionProfileBuffer();
	    }
	}
	Notifier _notifier = null;
	PathFollower follower = null;
	
	private void doPathFollowing(){
    	if (mStartingPath) {
    		mStartingPath = false;
    		follower = new PathFollower("x");
    		follower.initialize();
    	}
    	follower.execute();
    	if (follower.isFinished()){
    		setMotorLevels(0.0,0.0);
    		mLeftMaster.set(ControlMode.MotionMagic, follower.getFinalPositionLeft());
    		mRightMaster.set(ControlMode.MotionMagic, follower.getFinalPositionRight());
    	}else{
    		setMotorLevels(follower.getLeftMotorSetting(), follower.getRightMotorSetting());
    	}
	}
	

    	

    private Drive() {
        // Start all Talons in open loop mode.
        mLeftMaster = new WPI_TalonSRX(RobotMap.DRIVE_LEFT_TALON);
        mLeftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
        mLeftMaster.setSensorPhase(true);
        mLeftMaster.changeMotionControlFramePeriod(5);
        
		mLeftMaster.configNeutralDeadband(0.01, 10);

		mLeftMaster.configMotionProfileTrajectoryPeriod(10, 10); 

        _leftSlave1 = new WPI_VictorSPX(RobotMap.DRIVE_LEFT_VICTOR_1);
        _leftSlave1.follow(mLeftMaster);
        _leftSlave2 = new WPI_VictorSPX(RobotMap.DRIVE_LEFT_VICTOR_2);
        _leftSlave2.follow(mLeftMaster);
        
        mRightMaster = new WPI_TalonSRX(RobotMap.DRIVE_RIGHT_TALON);
        mRightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
        mRightMaster.changeMotionControlFramePeriod(5);
        mRightMaster.setSensorPhase(false);
        
		mRightMaster.configNeutralDeadband(0.01, 10);

		mRightMaster.config_kF(0, 0.275, 10);
		mRightMaster.config_kP(0, 0.25, 10);
		mRightMaster.config_kI(0, 0.0, 10);
		mRightMaster.config_kD(0, 0.0, 10);
		
		mLeftMaster.config_kF(0, 0.275, 10);
		mLeftMaster.config_kP(0, 0.25 , 10);
		mLeftMaster.config_kI(0, 0.0, 10);
		mLeftMaster.config_kD(0, 0.0, 10);
		
		mRightMaster.configNominalOutputForward(0, 10);
		mRightMaster.configNominalOutputReverse(0, 10);
		mRightMaster.configPeakOutputForward(1, 10);
		mRightMaster.configPeakOutputReverse(-1, 10);
		
		mLeftMaster.configNominalOutputForward(0, 10);
		mLeftMaster.configNominalOutputReverse(0, 10);
		mLeftMaster.configPeakOutputForward(1, 10);
		mLeftMaster.configPeakOutputReverse(-1, 10);

		mLeftMaster.configMotionCruiseVelocity(800, 10);
		mLeftMaster.configMotionAcceleration(400, 10);

		mRightMaster.configMotionCruiseVelocity(800, 10);
		mRightMaster.configMotionAcceleration(400, 10);
		
		mRightMaster.configMotionProfileTrajectoryPeriod(10, 10); 

        _rightSlave1 = new WPI_VictorSPX(RobotMap.DRIVE_RIGHT_VICTOR_1);
        _rightSlave1.follow(mRightMaster);
        _rightSlave2 = new WPI_VictorSPX(RobotMap.DRIVE_RIGHT_VICTOR_2);
        _rightSlave2.follow(mRightMaster);
        
        setOpenLoop(DriveSignal.NEUTRAL);

        // Path Following stuff
        mNavXBoard = new Gyro();

        // Force a CAN message across.
        mIsBrakeMode = true;
        setBrakeMode(false);

     //   mCSVWriter = new ReflectingCSVWriter<PathFollower.DebugOutput>("/home/lvuser/PATH-FOLLOWER-LOGS.csv",
     //           PathFollower.DebugOutput.class);
        
        _drive = new DifferentialDrive(mLeftMaster, mRightMaster);
    	_notifier = new Notifier(new PeriodicRunnable());
        _notifier.startPeriodic(0.005);
        
        
    }
    @Override
    public void registerEnabledLoops(Looper in) {
        in.register(mLoop);
    }

    /**
     * Configure talons for open loop control
     */
    public synchronized void setOpenLoop(DriveSignal signal) {
        if (mDriveControlState != DriveControlState.OPEN_LOOP) {
            mLeftMaster.set(ControlMode.PercentOutput, signal.getLeft());
            mRightMaster.set(ControlMode.PercentOutput, -signal.getRight());
            mDriveControlState = DriveControlState.OPEN_LOOP;
            setBrakeMode(false);
        }
        else
        {
	        mRightMaster.set(-signal.getRight());
	        mLeftMaster.set(signal.getLeft());
        }
    }

    public boolean isBrakeMode() {
        return mIsBrakeMode;
    }

    public synchronized void setBrakeMode(boolean on) {
        if (mIsBrakeMode != on) {
            mIsBrakeMode = on;
//            mRightMaster.enableBrakeMode(on);
//            mRightSlave.enableBrakeMode(on);
//            mLeftMaster.enableBrakeMode(on);
//            mLeftSlave.enableBrakeMode(on);
        }
    }

    @Override
    public synchronized void stop() {
        setOpenLoop(DriveSignal.NEUTRAL);
    }

    @Override
    public void outputToSmartDashboard() {
        SmartDashboard.putNumber("left percent output", mLeftMaster.getMotorOutputPercent());
        SmartDashboard.putNumber("right percent output", mRightMaster.getMotorOutputPercent());
        SmartDashboard.putNumber("left position (rotations)", mLeftMaster.getSelectedSensorPosition(0));///4096);
        SmartDashboard.putNumber("right position (rotations)", mRightMaster.getSelectedSensorPosition(0));///4096);
        SmartDashboard.putNumber("gyro pos", Gyro.getYaw());
    }

    public synchronized void resetEncoders() {
        /*mLeftMaster.setEncPosition(0);
        mLeftMaster.setPosition(0);
        mRightMaster.setPosition(0);
        mRightMaster.setEncPosition(0); 
        mLeftSlave.setPosition(0);
        mRightSlave.setPosition(0); */
    }

    @Override
    public void zeroSensors() {
        resetEncoders();
        mNavXBoard.reset();
    }

    public synchronized Gyro getNavXBoard() {
        return mNavXBoard;
    }

       @Override
    public void writeToLog() {
        //mCSVWriter.write();
    }
}
