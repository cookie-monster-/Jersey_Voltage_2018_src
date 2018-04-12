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

public class Drive extends Subsystem {
	
	private long startTime;

    private static Drive mInstance = null;
    private DifferentialDrive _drive;
    private String pathFilename;
    public void setPathFilename(String x){
    	pathFilename = x;
    }

    public static Drive getInstance() {
    	if ( mInstance == null ) {
    		synchronized ( Drive.class ) {
    			mInstance = new Drive();
    		}
    	}
    	return mInstance;
    }

    // The robot drivetrain's various states.
    public enum DriveControlState {
        OPEN_LOOP, // open loop voltage control
        PATH_FOLLOWING, // used for autonomous driving
        TEST_MODE, // to run the testSubsystem() method once, then return to OPEN_LOOP
    }
    public DriveControlState getState(){
    	return mDriveControlState;
    }

    public int getLeftEnc(){
    	return mLeftMaster.getSelectedSensorPosition(0);
    }
    public int getRightEnc(){
    	return mRightMaster.getSelectedSensorPosition(0);
    }
    
    double m_leftPathPos, m_rightPathPos;
    public void setPathPos(double leftPathPos,double rightPathPos){
    	synchronized(Drive.class){
    		m_leftPathPos = leftPathPos * Constants.kInchesPerTic / 12.0;
    		m_rightPathPos = rightPathPos * Constants.kInchesPerTic / 12.0;
    	}
    }
    public double getLeftPathPos(){
    	synchronized(Drive.class){
    		return m_leftPathPos;
    	}
    }
    public double getRightPathPos(){
    	synchronized(Drive.class){
    		return m_rightPathPos;
    	}
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
    private DebugOutput mDebugOutput;
    private final ReflectingCSVWriter<DebugOutput> mCSVWriter;
    
    private double mDrive,mTurn,mLastDrive,mLastTurn;
    
    public void startPath() {
    	System.out.println("in startPath");
    	synchronized (Drive.this) {
    		mDriveControlState = DriveControlState.PATH_FOLLOWING;
    		mStartingPath = true ;
    	}
    }
    
    public void runTest() {
    	System.out.println("in runTest");
    	synchronized (Drive.this) {
    		mDriveControlState = DriveControlState.TEST_MODE;
    	}
    }
    
    int iCall = 0;
    private final Loop mLoop = new Loop() {

        @Override
        public void onStart(double timestamp) {
            synchronized (Drive.this) {
            	startTime = System.nanoTime();
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
                	mDrive = OI.getInstance().getDrive();
                	mTurn = OI.getInstance().getTurn();
                	double liftHeight = Robot.getLift().getPos();
                	if(mDrive < mLastDrive){
                		if(liftHeight > 0 && liftHeight <= 1.5){
                			//mDrive = mLastDrive - Constants.kDriveMaxBackAccPerILiftLow;
                		}else if(liftHeight > 1.5){
                			//mDrive = mLastDrive - Constants.kDriveMaxBackAccPerILiftHigh;
                		}
                	}
                	_drive.arcadeDrive(mDrive, mTurn, false);//bool = squaredInputs
                    //mLeftMaster.setInverted(false);
                    invertRightSide(false);
                    _drive.setSafetyEnabled(true);
                    mLastDrive = mDrive;
                    mLastTurn = mTurn;
                    break;
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
                    break;
                case TEST_MODE:
                	testSubsystem();
                	mDriveControlState = DriveControlState.OPEN_LOOP;
                	break;
                default:
                    System.out.println("Unexpected drive control state: " + mDriveControlState);
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
    
    private void setMotorLevels(double left, double right){
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
    
	PathFollower follower = null;
	
	private void doPathFollowing(){
    	if (mStartingPath) {
    		mStartingPath = false;
    		setPathPos(0,0);
    		follower = new PathFollower(pathFilename);
    		System.out.println("follower != null");
    		follower.initialize();
    	}
    	if (follower.isFinished() == false){
    		System.out.println("doing path");
    		follower.execute();
    		setMotorLevels(follower.getLeftMotorSetting(), follower.getRightMotorSetting());
    	}else{
    		System.out.println("done with path");
    		setOpenLoop(DriveSignal.NEUTRAL);
    	}
    	
	}

	private Drive() {
        // Start all Talons in open loop mode.
        mLeftMaster = new WPI_TalonSRX(RobotMap.DRIVE_LEFT_TALON);
        mLeftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
        mLeftMaster.setSensorPhase(true);
        mLeftMaster.changeMotionControlFramePeriod(5);
        mLeftMaster.setNeutralMode(NeutralMode.Brake);
        
		mLeftMaster.configNeutralDeadband(0.01, 10);
		mLeftMaster.configMotionProfileTrajectoryPeriod(10, 10); 

        _leftSlave1 = new WPI_VictorSPX(RobotMap.DRIVE_LEFT_VICTOR_1);
        _leftSlave1.follow(mLeftMaster);
        _leftSlave1.setNeutralMode(NeutralMode.Brake);
        _leftSlave2 = new WPI_VictorSPX(RobotMap.DRIVE_LEFT_VICTOR_2);
        _leftSlave2.follow(mLeftMaster);
        _leftSlave2.setNeutralMode(NeutralMode.Brake);
        
        mRightMaster = new WPI_TalonSRX(RobotMap.DRIVE_RIGHT_TALON);
        mRightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
        mRightMaster.changeMotionControlFramePeriod(5);
        mRightMaster.setSensorPhase(false);
        mRightMaster.setNeutralMode(NeutralMode.Brake);
        
		mRightMaster.configNeutralDeadband(0.01, 10);

		mRightMaster.configMotionProfileTrajectoryPeriod(10, 10); 
		
        _rightSlave1 = new WPI_VictorSPX(RobotMap.DRIVE_RIGHT_VICTOR_1);
        _rightSlave1.follow(mRightMaster);
        _rightSlave1.setNeutralMode(NeutralMode.Brake);
        _rightSlave2 = new WPI_VictorSPX(RobotMap.DRIVE_RIGHT_VICTOR_2);
        _rightSlave2.follow(mRightMaster);
        _rightSlave2.setNeutralMode(NeutralMode.Brake);

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
        
        setOpenLoop(DriveSignal.NEUTRAL);

        // Path Following stuff
        mNavXBoard = new Gyro();

        // Force a CAN message across.
        mIsBrakeMode = true;
        setBrakeMode(false);

        mDebugOutput = new DebugOutput();
        mCSVWriter = new ReflectingCSVWriter<DebugOutput>("/home/lvuser/DriveLog.csv",
                DebugOutput.class);
        
        _drive = new DifferentialDrive(mLeftMaster, mRightMaster);
        
        
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
    
    private double convertEncoderToFeet(int encoder){
    	return encoder;
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
    
    public class DebugOutput{
    	public long sysTime;
    	public String driveMode;
    	public double gyroYaw;
    	public double leftEncoder;
    	public double rightEncoder;
    	public double leftEncoderVel;
    	public double rightEncoderVel;
    	public double leftMotorPercent;
    	public double rightMotorPercent;
    	public double leftMotorVoltage;
    	public double rightMotorVoltage;
    	public double leftMotorCurrent;
    	public double rightMotorCurrent;
    	public double driveStick;
    	public double turnStick;
    	public double leftPathPos;
    	public double leftPathVel;
    	public double leftPathAcc;
    	public double rightPathPos;
    	public double rightPathVel;
    	public double rightPathAcc;
    	public double pathHdg;
    	public int pathStep0;
    	public int pathStep1;
    	public double leftBusVoltage;
    	public double rightBusVoltage;
    	public double leftTemp;
    	public double rightTemp;
    	public boolean leftHasResetOccurred;
    	public boolean rightHasResetOccurred;
    	public boolean leftIsSafetyEnabled;
    	public boolean rightIsSafetyEnabled;
    }
    
    public void logValues(){
    	mDebugOutput.sysTime = System.nanoTime()-startTime;
    	mDebugOutput.driveMode = mDriveControlState.name();
    	mDebugOutput.gyroYaw = Gyro.getYaw();
    	mDebugOutput.leftEncoder = mLeftMaster.getSelectedSensorPosition(0);
    	mDebugOutput.rightEncoder = mRightMaster.getSelectedSensorPosition(0);
    	mDebugOutput.leftEncoderVel = mLeftMaster.getSelectedSensorVelocity(0)/10;//convert from ticks/100ms to ticks/10ms
    	mDebugOutput.rightEncoderVel = mRightMaster.getSelectedSensorVelocity(0)/10;
    	mDebugOutput.leftMotorPercent = mLeftMaster.getMotorOutputPercent();
    	mDebugOutput.rightMotorPercent = mRightMaster.getMotorOutputPercent();
    	mDebugOutput.leftMotorVoltage = mLeftMaster.getMotorOutputVoltage();
    	mDebugOutput.rightMotorVoltage = mRightMaster.getMotorOutputVoltage();
    	mDebugOutput.leftMotorCurrent = mLeftMaster.getOutputCurrent();
    	mDebugOutput.rightMotorCurrent = mRightMaster.getOutputCurrent();
    	if(mDriveControlState == DriveControlState.OPEN_LOOP){
	    	mDebugOutput.driveStick = OI.getInstance().getDrive();
	    	mDebugOutput.turnStick = OI.getInstance().getTurn();
	    	mDebugOutput.leftPathPos = 0;
	    	mDebugOutput.leftPathVel = 0;
	    	mDebugOutput.leftPathAcc = 0;
	    	mDebugOutput.rightPathPos = 0;
	    	mDebugOutput.rightPathVel = 0;
	    	mDebugOutput.rightPathAcc = 0;
	    	mDebugOutput.pathHdg = 0;
	    	mDebugOutput.pathStep0 = 0;
	    	mDebugOutput.pathStep1 = 0;
    	}else if(follower!=null){
    		mDebugOutput.driveStick = 0;
	    	mDebugOutput.turnStick = 0;
	    	mDebugOutput.leftPathPos = follower.getXLeft();
	    	mDebugOutput.leftPathVel = follower.getVLeft();
	    	mDebugOutput.leftPathAcc = follower.getALeft();
	    	mDebugOutput.rightPathPos = follower.getXRight();
	    	mDebugOutput.rightPathVel = follower.getVRight();
	    	mDebugOutput.rightPathAcc = follower.getARight();
	    	mDebugOutput.pathHdg = follower.getDesiredAngle();
	    	mDebugOutput.pathStep0 = follower.getStep0();
	    	mDebugOutput.pathStep1 = follower.getStep1();
    	}
    	mDebugOutput.leftBusVoltage = mLeftMaster.getBusVoltage();
    	mDebugOutput.rightBusVoltage = mRightMaster.getBusVoltage();
    	mDebugOutput.leftTemp = mLeftMaster.getTemperature();
    	mDebugOutput.rightTemp = mRightMaster.getTemperature();
    	mDebugOutput.leftHasResetOccurred = mLeftMaster.hasResetOccurred();
    	mDebugOutput.rightHasResetOccurred = mRightMaster.hasResetOccurred();
    	mDebugOutput.leftIsSafetyEnabled = mLeftMaster.isSafetyEnabled();
    	mDebugOutput.rightIsSafetyEnabled = mRightMaster.isSafetyEnabled();
    	mCSVWriter.add(mDebugOutput);
    }

    public synchronized void resetEncoders() {
    	mLeftMaster.setSelectedSensorPosition(0, 0, 10);
    	mRightMaster.setSelectedSensorPosition(0, 0, 10);
    }
    
    private void invertRightSide(boolean x){
    	mRightMaster.setInverted(x);
    	_rightSlave1.setInverted(x);
    	_rightSlave2.setInverted(x);
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
        mCSVWriter.write();
    }

   public boolean testSubsystem(){

	   boolean all_ok = false;
	   try{
		   FileWriter w = new FileWriter(new File("/home/lvuser/testLog.csv"));
	   all_ok = true;

	   _drive.setSafetyEnabled(false);
	   
	   mLeftMaster.setNeutralMode(NeutralMode.Coast);
	   _leftSlave1.setNeutralMode(NeutralMode.Coast);
	   _leftSlave2.setNeutralMode(NeutralMode.Coast);
	   mRightMaster.setNeutralMode(NeutralMode.Coast);
	   _rightSlave1.setNeutralMode(NeutralMode.Coast);
	   _rightSlave2.setNeutralMode(NeutralMode.Coast);

	   mLeftMaster.set(ControlMode.PercentOutput,0.0);
	   _leftSlave1.set(ControlMode.PercentOutput,0.0);
	   _leftSlave2.set(ControlMode.PercentOutput,0.0);
	   mRightMaster.set(ControlMode.PercentOutput,0.0);
	   _rightSlave1.set(ControlMode.PercentOutput,0.0);
	   _rightSlave2.set(ControlMode.PercentOutput,0.0);

	   //System.out.println("Right Master...");
	   //mRightMaster.setSelectedSensorPosition(0, 0, 10);
	   //mRightMaster.set(-0.5);
	   Timer.delay(0.5);
	   //double rightMaster_end = mRightMaster.getSelectedSensorPosition(0);
	   //double rightMaster_vel = mRightMaster.getSelectedSensorVelocity(0);
	   //mRightMaster.set(0);
	   
	   //System.out.println("... end="+rightMaster_end+",vel="+rightMaster_vel);
	   //Timer.delay(3.0);
	   
	   System.out.println("Right Master...");
	   mRightMaster.setSelectedSensorPosition(0, 0, 10);
	   mRightMaster.set(-0.5);
	   Timer.delay(3.0);
	   double rightMaster_end = mRightMaster.getSelectedSensorPosition(0);
	   double rightMaster_vel = mRightMaster.getSelectedSensorVelocity(0);
	   mRightMaster.set(0);
	   
	   w.write("rightMaster,"+rightMaster_end+","+rightMaster_vel+"\n");
	   System.out.println("... end="+rightMaster_end+",vel="+rightMaster_vel);
	   System.out.println("rightMasterDistError="+(rightMaster_end-Constants.kTestDistTarget)+" rightMasterVelError="+(rightMaster_vel-Constants.kTestVelTarget));
	   System.out.println("passed="+((Math.abs(rightMaster_end-Constants.kTestDistTarget)<Constants.kTestDistError)&&(Math.abs(rightMaster_vel-Constants.kTestVelTarget)<Constants.kTestVelError)));
	   Timer.delay(3.0);
	   
	   System.out.println("Right Slave 1...");
	   mRightMaster.setSelectedSensorPosition(0, 0, 10);
	   _rightSlave1.set(-0.5);
	   Timer.delay(3.0);
	   double rightSlave1_end = mRightMaster.getSelectedSensorPosition(0);
	   double rightSlave1_vel = mRightMaster.getSelectedSensorVelocity(0);
	   _rightSlave1.set(0);

	   w.write("_rightSlave1,"+rightSlave1_end+","+rightSlave1_vel+"\n");
	   System.out.println("... end="+rightSlave1_end+",vel="+rightSlave1_vel);
	   System.out.println("_rightSlave1DistError="+(rightSlave1_end-Constants.kTestDistTarget)+" _rightSlave1VelError="+(rightSlave1_vel-Constants.kTestVelTarget));
	   System.out.println("passed="+((Math.abs(rightSlave1_end-Constants.kTestDistTarget)<Constants.kTestDistError)&&(Math.abs(rightSlave1_vel-Constants.kTestVelTarget)<Constants.kTestVelError)));
	   Timer.delay(3.0);
	   
	   System.out.println("Right Slave 2...");
	   mRightMaster.setSelectedSensorPosition(0, 0, 10);
	   _rightSlave2.set(-0.5);
	   Timer.delay(3.0);
	   double rightSlave2_end = mRightMaster.getSelectedSensorPosition(0);
	   double rightSlave2_vel = mRightMaster.getSelectedSensorVelocity(0);
	   _rightSlave2.set(0);

	   w.write("_rightSlave2,"+rightSlave2_end+","+rightSlave2_vel+"\n");
	   System.out.println("... end="+rightSlave2_end+",vel="+rightSlave2_vel);
	   System.out.println("_rightSlave2DistError="+(rightSlave2_end-Constants.kTestDistTarget)+" _rightSlave2VelError="+(rightSlave2_vel-Constants.kTestVelTarget));
	   System.out.println("passed="+((Math.abs(rightSlave2_end-Constants.kTestDistTarget)<Constants.kTestDistError)&&(Math.abs(rightSlave2_vel-Constants.kTestVelTarget)<Constants.kTestVelError)));
	   Timer.delay(3.0);
	   
	   System.out.println("Left Master...");
	   mLeftMaster.setSelectedSensorPosition(0, 0, 10);
	   mLeftMaster.set(0.5);
	   Timer.delay(3.0);
	   double leftMaster_end = mLeftMaster.getSelectedSensorPosition(0);
	   double leftMaster_vel = mLeftMaster.getSelectedSensorVelocity(0);
	   mLeftMaster.set(0);

	   w.write("leftMaster,"+leftMaster_end+","+leftMaster_vel+"\n");
	   System.out.println("... end="+leftMaster_end+",vel="+leftMaster_vel);
	   System.out.println("leftMasterDistError="+(leftMaster_end-Constants.kTestDistTarget)+" leftMasterVelError="+(leftMaster_vel-Constants.kTestVelTarget));
	   System.out.println("passed="+((Math.abs(leftMaster_end-Constants.kTestDistTarget)<Constants.kTestDistError)&&(Math.abs(leftMaster_vel-Constants.kTestVelTarget)<Constants.kTestVelError)));
	   Timer.delay(3.0);
	   
	   System.out.println("Left Slave 1...");
	   mLeftMaster.setSelectedSensorPosition(0, 0, 10);
	   _leftSlave1.set(0.5);
	   Timer.delay(3.0);
	   double leftSlave1_end = mLeftMaster.getSelectedSensorPosition(0);
	   double leftSlave1_vel = mLeftMaster.getSelectedSensorVelocity(0);
	   _leftSlave1.set(0);

	   w.write("_leftSlave1,"+leftSlave1_end+","+leftSlave1_vel+"\n");
	   System.out.println("... end="+leftSlave1_end+",vel="+leftSlave1_vel);
	   System.out.println("leftSlave1DistError="+(leftSlave1_end-Constants.kTestDistTarget)+" leftSlave1VelError="+(leftSlave1_vel-Constants.kTestVelTarget));
	   System.out.println("passed="+((Math.abs(leftSlave1_end-Constants.kTestDistTarget)<Constants.kTestDistError)&&(Math.abs(leftSlave1_vel-Constants.kTestVelTarget)<Constants.kTestVelError)));
	   Timer.delay(3.0);
	   
	   System.out.println("Left Slave 2...");
	   mLeftMaster.setSelectedSensorPosition(0, 0, 10);
	   _leftSlave2.set(0.5);
	   Timer.delay(3.0);
	   double leftSlave2_end = mLeftMaster.getSelectedSensorPosition(0);
	   double leftSlave2_vel = mLeftMaster.getSelectedSensorVelocity(0);
	   _leftSlave2.set(0);

	   w.write("_leftSlave2,"+leftSlave2_end+","+leftSlave2_vel+"\n");
	   System.out.println("... end="+leftSlave2_end+",vel="+leftSlave2_vel);
	   System.out.println("leftSlave2DistError="+(leftSlave2_end-Constants.kTestDistTarget)+" leftSlave2VelError="+(leftSlave2_vel-Constants.kTestVelTarget));
	   System.out.println("passed="+((Math.abs(leftSlave2_end-Constants.kTestDistTarget)<Constants.kTestDistError)&&(Math.abs(leftSlave2_vel-Constants.kTestVelTarget)<Constants.kTestVelError)));
	   
	   
	   
	   _leftSlave1.follow(mLeftMaster);
	   _leftSlave2.follow(mLeftMaster);
	   _rightSlave1.follow(mRightMaster);
	   _rightSlave2.follow(mRightMaster);
	   
	   mLeftMaster.setNeutralMode(NeutralMode.Brake);
	   _leftSlave1.setNeutralMode(NeutralMode.Brake);
	   _leftSlave2.setNeutralMode(NeutralMode.Brake);
	   mRightMaster.setNeutralMode(NeutralMode.Brake);
	   _rightSlave1.setNeutralMode(NeutralMode.Brake);
	   _rightSlave2.setNeutralMode(NeutralMode.Brake);

	   // Let the onLoop() method enable safety mode again...
	   w.close();
	   }catch(Exception e){}
	   return all_ok;
   }       
}
