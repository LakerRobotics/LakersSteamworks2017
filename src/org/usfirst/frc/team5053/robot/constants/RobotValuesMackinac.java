package org.usfirst.frc.team5053.robot.constants;

import org.usfirst.frc.team5053.robot.RobotConstants;

public class RobotValuesMackinac extends RobotValues {
	public static double getLeftEncoderDistancePerPulse(){return 6*Math.PI/1024;};//converts click to distance in inches
	public static double getRightEncoderDistancePerPulse(){return 6*Math.PI/1024;};//converts click to distance in inches
	public static double getShooterEncoderDistancePerPulse(){return (60/1024)*(72/24);};// convert 1 click/sec to RPM(rev/min): 1 Click/sec * 60sec/min * 1rev/1024clicks = rev/min
                                                           //                                    : 72 teeth / rev of 2nd axle * (1 rev of primary axle/24teeth)
	public static double getPidDist_P(){return 0.1;};
	public static double getPidDist_I(){return 0.0;};
	public static double getPidDist_D(){return 0.0;};
	public static double getPidDist_AbsoluteTolerance(){return 1.0;};

}
