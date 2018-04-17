package org.usfirst.frc.team4587.robot.commands;

import org.usfirst.frc.team4587.robot.Constants;
import org.usfirst.frc.team4587.robot.subsystems.Intake.IntakeControlState;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class StupidAuto extends CommandGroup {

    public StupidAuto() {
    	addSequential(new DelayTime(1));
    	addSequential(new FollowPath("anyToCrossLine"));
    }
}
