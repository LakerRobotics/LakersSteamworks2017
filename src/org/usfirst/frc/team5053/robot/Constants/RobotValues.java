package org.usfirst.frc.team5053.robot.Constants;

import org.usfirst.frc.team5053.robot.RobotConstants;

/*
 * This is a base orginzational class that provides the pattern for a class that holds all the robot specific setting
 * it is expected there will be a LilGeek, Machinac and possibly Rudy implementation of this class
 */
public abstract class RobotValues {
	
	public  boolean getRightDriveInverted(){return false;};
	public  boolean getLeftDriveInverted(){return false;};
	
	public  int getIntakePWM(){return 0;};//2);
	public  int getIndexerPWM(){return 0;};//3);
	public  int getShooterPWM(){return 0;};

	public  double getLeftEncoderDistancePerPulse(){return 0.0;};//converts click to distance in inches
	public  double getRightEncoderDistancePerPulse(){return 0.0;};//converts click to distance in inches
	public  double getShooterEncoderDistancePerPulse(){return 0.0;};//converts click to RPM

	// Drive Train motion Control distance
	public  double getPidDist_P(){return 0.0;};
	public  double getPidDist_I(){return 0.0;};
	public  double getPidDist_D(){return 0.0;};
	public  double getPidDist_AbsoluteTolerance(){return 0.0;};
		
	public  double getMaxRotationSpeed(){return 0.0;};
		
//		public static int getDriverForwardPowerAxis(){ return 0;};
//		public static int getDriverRotationPowerAxis(){ return 0;};
		
}
