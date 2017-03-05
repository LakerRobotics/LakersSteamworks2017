package org.usfirst.frc.team5053.robot.Constants;

public class RobotValuesLilGeek extends RobotValuesMackinac {
	
	public  boolean getRightDriveInverted(){return false;};
	public  boolean getLeftDriveInverted(){return false;};

	
	public  double getLeftEncoderDistancePerPulse(){return 6*Math.PI/360;};//converts click to distance in inches
	public  double getRightEncoderDistancePerPulse(){return 6*Math.PI/360;};//converts click to distance in inches
	public  double getPidDist_AbsoluteTolerance(){return 5.2;};
	
//	public static int getDriverForwardPowerAxis(){ return 2;};
//	public static int getDriverRotationPowerAxis(){ return 1;};

}
