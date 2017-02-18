package org.usfirst.frc.team5053.robot;

import org.usfirst.frc.team5053.robot.constants.RobotValues;
import org.usfirst.frc.team5053.robot.constants.RobotValuesMackinac;
import org.usfirst.frc.team5053.robot.constants.RobotValuesLilGeek;
import org.usfirst.frc.team5053.robot.constants.RobotValuesRudy;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public  class RobotConstants {
	private static RobotValues robotWeAreRunningOn;
	private static String robotName;
	
	static{
    	robotName = SmartDashboard.getString("robotName","LilGeek");//Default to "Mackinaw" if don't get a value
    	if("Mackinaw"==robotName){ robotWeAreRunningOn = new RobotValuesMackinac();};
    	if("LilGeek"==robotName){ robotWeAreRunningOn = new RobotValuesLilGeek();};
    	if("Rudy"==robotName){ robotWeAreRunningOn = new RobotValuesRudy();};
	}
	public static int getIntakePWM(){return robotWeAreRunningOn.getIntakePWM();};//2);
	public static int getIndexerPWM(){return robotWeAreRunningOn.getIndexerPWM();};//3);
	public static int getShooterPWM(){return robotWeAreRunningOn.getShooterPWM();};
	
	public static double getLeftEncoderDistancePerPulse(){return robotWeAreRunningOn.getLeftEncoderDistancePerPulse();};
	public static double getRightEncoderDistancePerPulse(){return robotWeAreRunningOn.getRightEncoderDistancePerPulse();};
	public static  double getShooterEncoderDistancePerPulse(){return robotWeAreRunningOn.getShooterEncoderDistancePerPulse();};
	
	public static  double getPidDist_P(){return robotWeAreRunningOn.getPidDist_P();};
	public static  double getPidDist_I(){return robotWeAreRunningOn.getPidDist_I();};
	public static  double getPidDist_D(){return robotWeAreRunningOn.getPidDist_D();};
	public static  double getPidDist_AbsoluteTolerance(){return robotWeAreRunningOn.getPidDist_AbsoluteTolerance();};

}
