package org.usfirst.frc.team5053.robot.Constants;

public class RobotValuesMackinac extends RobotValues {
	
	public  boolean getRightDriveInverted(){return true;};
	public  boolean getLeftDriveInverted(){return true;};
	
	public  int getIntakePWM(){return 2;};
	public  int getIndexerPWM(){return 3;};
	public  int getShooterPWM(){return 4;};
	
	public  double getLeftEncoderDistancePerPulse(){return 6*Math.PI/1024;};//converts click to distance in inches
	public  double getRightEncoderDistancePerPulse(){return 6*Math.PI/1024;};//converts click to distance in inches
	public  double getShooterEncoderDistancePerPulse(){return (60/1024)*(72/24);};// convert 1 click/sec to RPM(rev/min): 1 Click/sec * 60sec/min * 1rev/1024clicks = rev/min
                                                           //                                    : 72 teeth / rev of 2nd axle * (1 rev of primary axle/24teeth)
	public  double getPidDist_P(){return 0.1;};
	public  double getPidDist_I(){return 0.0;};
	public  double getPidDist_D(){return 0.0;};
	public  double getPidDist_AbsoluteTolerance(){return 1.0;};
	
	public  double getMaxRotationSpeed(){return 100;};
	
//	public static int getDriverForwardPowerAxis(){ return 1;};
//	public static int getDriverRotationPowerAxis(){ return 5;};

}
