package org.usfirst.frc.team5053.robot.Constants;

public class RobotValuesRudy extends RobotValuesMackinac {
	
	public  int getIntakePWM(){return 4;};
	public  int getIndexerPWM(){return 7;};//  no-op -- nothing is connected to 7
	public  int getShooterPWM(){return 2;};//Left shooter
	
	public  double getLeftEncoderDistancePerPulse(){return 8*Math.PI/1024;};//converts click to distance in inches
	public  double getRightEncoderDistancePerPulse(){return 8*Math.PI/1024;};//converts click to distance in inches
	public  double getShooterEncoderDistancePerPulse(){return (60/20);};// convert 1 click/sec to RPM(rev/min): 1 Click/sec * 60sec/min * 1rev/20clicks = rev/min

}
