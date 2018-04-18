package org.usfirst.frc.team4587.robot;

import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Waypoint;

import java.util.Arrays;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.videoio.VideoCapture;
import org.usfirst.frc.team4587.robot.commands.FollowPath;
import org.usfirst.frc.team4587.robot.commands.ScaleAuto;
import org.usfirst.frc.team4587.robot.commands.SetLiftArmSetpoints;
import org.usfirst.frc.team4587.robot.commands.StupidAuto;
import org.usfirst.frc.team4587.robot.commands.SwitchAuto;
//import org.usfirst.frc.team4587.robot.commands.StartMatchSwitchAuto;
import org.usfirst.frc.team4587.robot.loops.Looper;
import org.usfirst.frc.team4587.robot.paths.PathReader;
import org.usfirst.frc.team4587.robot.paths.PathWriter;
import org.usfirst.frc.team4587.robot.subsystems.Drive;
import org.usfirst.frc.team4587.robot.subsystems.Drive.DriveControlState;
import org.usfirst.frc.team4587.robot.subsystems.Intake.IntakeControlState;
import org.usfirst.frc.team4587.robot.subsystems.Intake;
import org.usfirst.frc.team4587.robot.subsystems.Lift;
import org.usfirst.frc.team4587.robot.subsystems.Tines;
import org.usfirst.frc.team4587.robot.util.CrashTracker;
import org.usfirst.frc.team4587.robot.util.DriveSignal;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends TimedRobot {

	// The SubsystemManager handles logging and looping for all registered subsystems.
	// I think it would be better to have the SubsystemManager own the looper and control all interactions
	// with the subsystems, but for now this is OK.
	private SubsystemManager mSubsystemManager = null;
	private Looper mEnabledLooper = null;

	// The subsystem that manages the drive base.
	// Again, it would be better for SubsystemManager to control the interactions with the subsystem.
	public static Drive getDrive(){
		return Drive.getInstance();
	}
	public static Lift getLift(){
		return Lift.getInstance();
	}
	public static Intake getIntake(){
		return Intake.getInstance();
	}
	public static Tines getTines(){
		return Tines.getInstance();
	}
	private static PowerDistributionPanel m_PDP;
	public static PowerDistributionPanel getPDP(){
		return m_PDP;
	}
	 
	/**
	 * Constructor
	 */
	public Robot() {
		CrashTracker.logRobotConstruction();
	}

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	private boolean m_robotInit_loggedError = false;
	@Override
	public void robotInit() {
		try {
			CrashTracker.logRobotInit();
		    m_PDP = new PowerDistributionPanel(0);
			// Create all subsystems and register them with the subsystem manager.
			mEnabledLooper = new Looper();
			mSubsystemManager = new SubsystemManager(Arrays.asList(Drive.getInstance(),Lift.getInstance(),Intake.getInstance(),Tines.getInstance()));
		    mSubsystemManager.registerEnabledLoops(mEnabledLooper);
			// Initialize the Operator Interface
			OI.getInstance();
			
		    //CameraServer.getInstance().startAutomaticCapture();
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t,"robotInit");
			if ( m_robotInit_loggedError == false ) {
				m_robotInit_loggedError = true ;
				System.out.println("robotInit Crash: "+t.toString());
			}
		}
	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */
	private boolean m_disabledInit_loggedError = false;
	@Override
	public void disabledInit() {
		try {
			CrashTracker.logDisabledInit();

			// Stop all subsystem loops.
			mEnabledLooper.stop();

			// Call stop() on all our registered Subsystems.
			mSubsystemManager.stop();

		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t,"disabledInit");
			if ( m_disabledInit_loggedError == false ) {
				m_disabledInit_loggedError = true;
				System.out.println("disabledInit Crash: "+t.toString());
			}
		}
	}

	/**
	 * This function is called periodically while the robot is in Disabled mode.
	 */
	private boolean m_disabledPeriodic_loggedError = false;
	@Override
	public void disabledPeriodic() {
		try {
			SmartDashboard.putNumber("count0", OI.getInstance().getCount0());
			SmartDashboard.putNumber("count1", OI.getInstance().getCount1());
			SmartDashboard.putNumber("count2", OI.getInstance().getCount2());
			SmartDashboard.putNumber("count3", OI.getInstance().getCount3());
			SmartDashboard.putBoolean("tines switch", OI.getInstance().getTinesSwitch());
			SmartDashboard.putNumber("POV", OI.getInstance().getPOV());
			allPeriodic();
			setGm();
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t,"disabledPeriodic");
			if ( m_disabledPeriodic_loggedError == false ) {
				m_disabledPeriodic_loggedError = true;
				System.out.println("disabledPeriodic Crash: "+t.toString());
			}
		}
	}

	/**
	 * This function is called once each time the robot enters Autonomous mode.
	 * You can use it to set the subsystems up to run the autonomous commands.
	 */
	Command autonomousCommand;
	private void setAutoCommand(){
		int count0 = OI.getInstance().getCount0();
		switch(count0){
		case 0:
			autonomousCommand = new StupidAuto();
			break;
		case 1:
			//autonomousCommand = new LeftScaleAuto();
			break;
		case 2:
			autonomousCommand = new SwitchAuto();
			break;
		case 3:
			//autonomousCommand = new RightScaleAuto();
			break;
		default:
			autonomousCommand = new StupidAuto();
			break;
			
		}
		SmartDashboard.putString("autonomousCommand", autonomousCommand.getName());
	}
	private static String m_gm;
	public static String getGm(){
		return m_gm;
	}
	private void setGm(){
		m_gm = DriverStation.getInstance().getGameSpecificMessage();
		SmartDashboard.putString("GM", m_gm);
	}
	private boolean m_autonomousInit_loggedError = false;
	private static boolean mInTeleop = false;
	public static boolean getInTeleop(){
		return mInTeleop;
	}
	static int pathsRan = 0;
	public static int getPathsRan(){
		return pathsRan;
	}
	int delayCount;
	@Override
	public void autonomousInit() {
		try {
			mInTeleop = false;
			countForGm = 0;
			CrashTracker.logAutonomousInit();

			// Start the subsystem loops.
			mEnabledLooper.start();
			if(m_gm.length()==0){
				setGm();
			}
			if(m_gm.length()>0){
				//autonomousCommand.start();
			}
			pathsRan = 0;
			delayCount=0;
			
			
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t,"autonomousInit");
			if ( m_autonomousInit_loggedError == false ) {
				m_autonomousInit_loggedError = true;
				System.out.println("autonomousInit Crash: "+t.toString());
			}
		}
	}

	/**
	 * This function is called periodically while the robot is in Autonomous mode.
	 */
	private boolean m_autonomousPeriodic_loggedError = false;
	private int countForGm;
	@Override
	public void autonomousPeriodic() {
		try {
			allPeriodic();
			//String gm = DriverStation.getInstance().getGameSpecificMessage();
			if(/*gm.length() > 0 && */pathsRan == 0){
				//Command autonomousCommand = new ScaleAuto(gm);
				//Command autonomousCommand = new SwitchAuto();
				//Command autonomousCommand = new StupidAuto();
				Command autonomousCommand = new FollowPath("centerToLeftSwitch");
				autonomousCommand.start();
				pathsRan = 1;
			}
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t,"autonomousPeriodic");
			if ( m_autonomousPeriodic_loggedError == false ) {
				m_autonomousPeriodic_loggedError = true;
				System.out.println("autonomousPeriodic Crash: "+t.toString());
			}
		}
	}

	/**
	 * This function is called once each time the robot enters Teleop mode.
	 * You can use it to set the subsystems up to run under operator control.
	 */
	private boolean m_teleopInit_loggedError = false;
	@Override
	public void teleopInit() {
		try {
			mInTeleop = true;
			CrashTracker.logTeleopInit();

			// Start the subsystem loops.
			mEnabledLooper.start();
			//Command autonomousCommand = new StartMatchReZeroLiftArm();
			//autonomousCommand.start();

			// Change the Drive subsystem to manual control.
			getDrive().setOpenLoop(DriveSignal.NEUTRAL);

		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t,"teleopInit");
			if ( m_teleopInit_loggedError == false ) {
				m_teleopInit_loggedError = true;
				System.out.println("teleopInit Crash: "+t.toString());
			}
		}
	}

	/**
	 * This function is called periodically while the robot is in Teleop mode.
	 */
	private boolean m_teleopPeriodic_loggedError = false;
	@Override
	public void teleopPeriodic() {
		try {
			allPeriodic();
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t,"teleopPeriodic");
			if ( m_teleopPeriodic_loggedError == false ) {
				m_teleopPeriodic_loggedError = true;
				System.out.println("teleopPeriodic Crash: "+t.toString());
			}
		}
	}

	/**
	 * This function is called once each time the robot enters Test mode.
	 * You can use it to set the subsystems up to run their self-test routines.
	 */
	private boolean m_testInit_loggedError = false;
	@Override
	public void testInit() {
		try {
			CrashTracker.logTestInit();

			// ===== TEMPORARY CODE - REMOVE THIS =====
	        mEnabledLooper.start();
	        getDrive().runTest();
	        // ========================================

			// TODO...
	        // ... Start a separate thread that runs through the self-test for each registered subsystem.
	        // ... Create and manage the thread in SubsystemManager.

		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t,"testInit");
			if ( m_testInit_loggedError == false ) {
				m_testInit_loggedError = true;
				System.out.println("testInit Crash: "+t.toString());
			}
		}
	}

	/**
	 * This function is called periodically while the robot is in Test mode.
	 */
	private boolean m_testPeriodic_loggedError = false;
	@Override
	public void testPeriodic() {
		try {
			allPeriodic();
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t,"testPeriodic");
			if ( m_testPeriodic_loggedError == false ) {
				m_testPeriodic_loggedError = true;
				System.out.println("testPeriodic Crash: "+t.toString());
			}
		}
	}
	
	/*
	 * This is the method called periodically during every periodic mode.
	 * It runs all the logging methods, and then runs the WPI scheduler.
	 */
	public void allPeriodic() {
		mSubsystemManager.outputToSmartDashboard();
		mSubsystemManager.writeToLog();
		mEnabledLooper.outputToSmartDashboard();
		Scheduler.getInstance().run();
	}
}