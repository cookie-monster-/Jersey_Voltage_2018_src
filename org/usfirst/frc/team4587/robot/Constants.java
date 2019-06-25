package org.usfirst.frc.team4587.robot;

import edu.wpi.first.wpilibj.Solenoid;

import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.Enumeration;
import java.util.HashMap;

/**
 * A list of constants used by the rest of the robot code. This include physics constants as well as constants
 * determined through calibrations.
 */
public class Constants{
    public static double kLooperDt = 0.005;

    public static double kSensorUnitsPerRotation = 4096;
    
    public static double kStepSizeSeconds = 0.01;
    public static double kMaxFeetPerSecond = 10.0;
    public static double kMaxAcceleration = 15.0;
    public static double kMaxJerk = 40.0;
    public static double kWheelBaseFeet = 25.75 / 12.0;
    
    public static final double kLiftAMax = 20;
    public static final double kLiftVMax = 20;

    public static final double kScaleHighLiftFlip = 3.1;
    public static final double kScaleLowLiftFlip = 1.0;
    public static final double kScaleLiftNoFlip = 3.1;
    public static final double kScaleHighArm = -147.0;
    public static final double kScaleLowArm = -185.0;
    public static final double kScaleArmFlip = -12.0;

    /* ROBOT PHYSICAL CONSTANTS */

    // Wheels
    public static final double kDriveWheelDiameterInches = 6;
    public static double kTrackWidthInches = 26.655;
    public static double kTrackScrubFactor = 0.924;
    public static final double kInchesPerTic = Constants.kDriveWheelDiameterInches * Math.PI / Constants.kSensorUnitsPerRotation;

    public static final double kDriveMaxBackAccPerILiftLow = 0.02;
    public static final double kDriveMaxBackAccPerILiftHigh = 0.01;
    public static final double kPathDoneTicsTolerance = 4 / kInchesPerTic;
    
    //Lift
    public static final double kLiftTicsPerRev = 256.0;
    public static final double kLiftInchesPerRevHighGear = 2.5 * Math.PI * (26.0/84.0) * (50.0/34.0); // 40 to 44, 24 to 84, 2.5in dia
    public static final double kLiftInchesPerTicHighGear = Constants.kLiftInchesPerRevHighGear / Constants.kLiftTicsPerRev;//2635
    public static final double kLiftInchesPerRevLowGear = 2.5 * Math.PI * (26.0/84.0) * (24.0/60.0); // 40 to 44, 24 to 84, 2.5in dia
    public static final double kLiftInchesPerTicLowGear = Constants.kLiftInchesPerRevLowGear / Constants.kLiftTicsPerRev;
    public static final double kLiftMaxAmps = 45;
    public static final long kLiftTimeSinceHitMax = 1000*1000*1000;
    public static final double kLiftMaxMotorUp = 1.0;
    public static final double kLiftMaxMotorDown = -0.6;//0.4
    public static final double kLiftSlowMotorUp = 0.3;
    public static final double kLiftSlowMotorDown = -0.5;//0.3
    public static final boolean kLiftBrakeOn = false;
    public static final boolean kLiftBrakeOff = true;
    public static final boolean kLiftClimbOn = true;
    public static final boolean kLiftClimbOff = false;
    public static final long kLiftBrakeTimeToRelease = 1000*1000*100;//100ms
    public static final double kLiftJoystickDeadband = 0.1;
    public static final double kLiftSoftStopHigh = 3.1;
    public static final double kLiftSoftStopLow = -1.83;//-1.75
    public static final double kLiftSoftStopForArm = 0.75;
    public static final double kLiftTolerance = 0.1;
    public static final double kLiftVelFPI = 0.06;
    public static final double kMinDistanceUnsafeArm = 0.75;
    public static final double kBearingPos = 1.5;
    public static final double kFlipPos = 0.4;
    public static final double kGravityEffectMotorLevel = 0.3;
    public static final double kLiftFlooperHeight = 0.5;
    public static final double kLiftHeightTolerance = 0.1;
    public static final double kLiftBumpDist = 0.5;
    public static final double kLiftClimbHeight = 2.275;

    //Arm
    public static final double kArmTicsPerRev = 1024.0;
    public static final double kArmDegreesPerRev = 360.0 * (30.0/84.0); // 30 to 84
    public static final double kArmDegreesPerTic = Constants.kArmDegreesPerRev / Constants.kArmTicsPerRev; //-425
    public static final double kArmMaxAmps = 45;
    public static final long kArmTimeSinceHitMax = 1000*1000*1000;
    public static final double kArmMaxMotorUp = 0.8;
    public static final double kArmMaxMotorDown = -1.0;///-0.65
    public static final double kArmSlowMotorUp = 0.35;//3
    public static final double kArmSlowMotorDown = -0.2;//2
    public static final double kArmJoystickDeadband = 0.1;
    public static final double kArmSoftStopHigh = 15.0;
    public static final double kArmSoftStopLow = -195.0;
    public static final double kArmSoftStopMiddle = -60.0;
    public static final double kArmSoftStopLifting = -182.0;
    public static final double kArmSoftStopLiftingTolerance = 10;//degrees
    public static final double kArmRotationRPI = 2;//?
    public static final double kArmFlooperDeg = -183.5;
    public static final double kArmIntakeDeg = -183.5;
    public static final double kArmDegTolerance = 2.0;
    public static final double kArmClimbDeg = 0.0;
    
    //Intake
    public static final double kIntakeOutSlow = -0.55;
    public static final double kIntakeOutFast = -0.7;
    public static final double kIntakeOutSwitchFast = -0.85;
    public static final double kIntakeIn = 1.0;
    public static final double kIntakeHold = 0.25;//0.1
    //public static final double kIntakeInMedium = 0.5;
    public static final double kIntakeCubeDistInches = 10;//8
    public static final double kIntakeCurrentLimit = 15;//20 amp breaker 
    public static final boolean kIntakeCloseOn = false;
    public static final boolean kIntakeCloseOff = true;
    public static final boolean kIntakeOpenOn = true;
    public static final boolean kIntakeOpenOff = false;
    
    //Tines
    public static final double kTinesMotorLevelPerIntervalUp = 0.01;
    public static final double kTinesMotorLevelPerIntervalDown = 0.01;
    
    // Geometry
    public static double kCenterToFrontBumperDistance = 16.33;
    public static double kCenterToIntakeDistance = 23.11;
    public static double kCenterToRearBumperDistance = 16.99;
    public static double kCenterToSideBumperDistance = 17.225;

    /* CONTROL LOOP GAINS */
    
    public static final double kODesiredFinalMotor = 0.8;
    public static final double kSimpleArcTolerance = 10.0;

    public static final double kPathFollowKa = 0.1;//0.0001
    public static final double kPathFollowKv = 0.0029;//0.000275
    public static final double kPathFollowKp = 0.00005;//0.00025
    public static final double kPathFollowKg = 0.01;//0.015

    public static final double kPathHoldKp = 0.0005;
    public static final double kPathHoldKg = 0.0001;

    public static final double kLiftKp = 0.02;
    public static final double kLiftKa = 0.015;
    public static final double kLiftKv = 0.15;

    public static final double kLiftHoldKp = 0.05;
    public static final double kLiftHoldKi = 0.0;
    public static final double kLiftHoldKd = 0.0;
    public static final double kLiftHoldLowPower = 0.0;
    public static final double kLiftHoldHighPower = 0.14;
    
    public static final double kArmHoldKp = 0.15;
    public static final double kArmHoldKi = 0.0;
    public static final double kArmHoldKd = 0.0;
    public static final double kArmHoldPower = 0.4;

    public static final double kTestVelTarget = 1700;
    public static final double kTestDistTarget = 50000;
    public static final double kTestVelError = 250;
    public static final double kTestDistError = 6000;
    
}
