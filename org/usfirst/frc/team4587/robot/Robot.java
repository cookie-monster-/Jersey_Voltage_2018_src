package org.usfirst.frc.team4587.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Waypoint;

import java.io.FileWriter;
import java.util.Arrays;

import org.usfirst.frc.team4587.robot.loops.Looper;
import org.usfirst.frc.team4587.robot.paths.PathReader;
import org.usfirst.frc.team4587.robot.paths.PathWriter;
import org.usfirst.frc.team4587.robot.subsystems.Drive;
import org.usfirst.frc.team4587.robot.subsystems.TestTheSparks;
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

	 private SubsystemManager mSubsystemManager = null;
	 private Looper mEnabledLooper = null;
	 private static Drive mDrive = null;
	 public static Drive getDrive(){
		 return mDrive;
	 }
	 private static TestTheSparks mTestTheSparks = null;
	 public static TestTheSparks getTestTheSparks(){
		 return mTestTheSparks;
	 }
	 private static PathReader mTestPath;
	 public static PathReader getTestPath(){
		 return mTestPath;
	 }
	 private static FileWriter writer;
	 public static void writeToFile(String x){
		 try{
		 writer.write(x);
		 }catch(Exception e){}
	 }
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	public Robot() {
	CrashTracker.logRobotConstruction();
	}
	@Override
	public void robotInit() {
		mEnabledLooper = new Looper();
		mSubsystemManager = new SubsystemManager(Arrays.asList(Drive.getInstance()));
		mDrive = Drive.getInstance();
		mTestTheSparks = new TestTheSparks();
		OI.getInstance();
		try {
			CrashTracker.logRobotInit();
		    mSubsystemManager.registerEnabledLoops(mEnabledLooper);
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
		    throw t;
		}
		
		Waypoint[] points = new Waypoint[] {
				new Waypoint(0, 0, 0),
				new Waypoint(0.25, 0, 0),
			    new Waypoint(102.0/12.0, 55.5/12.0, 0),
			};
		PathWriter.writePath("test", points, false/*isReversed*/);
		mTestPath = new PathReader("test");
		RobotMap.printPortList();
	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */
	@Override
	public void disabledInit() {
	      try {
	            CrashTracker.logDisabledInit();

	            mEnabledLooper.stop();

	            // Call stop on all our Subsystems.
	            mSubsystemManager.stop();

	            mDrive.setOpenLoop(DriveSignal.NEUTRAL);
	            
	        } catch (Throwable t) {
	            CrashTracker.logThrowableCrash(t);
	            throw t;
	        }
	      try{
	      writer.close();
	      }catch(Exception e){}
	}

	@Override
	public void disabledPeriodic() {
        allPeriodic();
      
	}

	@Override
	public void autonomousInit() {
		mEnabledLooper.start();
		mDrive.startPath();
	}

	@Override
	public void autonomousPeriodic() {
		allPeriodic();
	}

	@Override
	public void teleopInit() {
		try {
            CrashTracker.logTeleopInit();

            // Start loopers
            mEnabledLooper.start();
            mDrive.setOpenLoop(DriveSignal.NEUTRAL);
            //mDrive.setBrakeMode(false);
            // Shift to high
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
		try{
	    writer = new FileWriter("/home/lvuser/PathLog.csv");
	    }catch(Exception e){}
	}

	@Override
	public void teleopPeriodic() {
        try {
            double timestamp = Timer.getFPGATimestamp();
            
            allPeriodic();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
	}

	@Override
	public void testInit() {
		System.out.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        mEnabledLooper.start();
        mDrive.runTest();
	}
	
	@Override
	public void testPeriodic() {
		allPeriodic();
	}
	 public void allPeriodic() {
	        mSubsystemManager.outputToSmartDashboard();
	        mSubsystemManager.writeToLog();
	        mEnabledLooper.outputToSmartDashboard();
			Scheduler.getInstance().run();
}
	}
	