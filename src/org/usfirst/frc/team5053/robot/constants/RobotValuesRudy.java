package org.usfirst.frc.team5053.robot.constants;

public class RobotValuesRudy extends RobotValuesMackinac {
	public static double getLeftEncoderDistancePerPulse(){return 8*Math.PI/1024;};//converts click to distance in inches
	public static double getRightEncoderDistancePerPulse(){return 8*Math.PI/1024;};//converts click to distance in inches
	public static double getShooterEncoderDistancePerPulse(){return (60/20);};// convert 1 click/sec to RPM(rev/min): 1 Click/sec * 60sec/min * 1rev/20clicks = rev/min

}
