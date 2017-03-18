package org.usfirst.frc.team5053.robot;

import org.usfirst.frc.team5053.robot.Constants.RobotValues;
import org.usfirst.frc.team5053.robot.Constants.RobotValuesMackinac;
import org.usfirst.frc.team5053.robot.Constants.RobotValuesLilGeek;
import org.usfirst.frc.team5053.robot.Constants.RobotValuesRudy;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotConstants 
{
	private static RobotValues robotWeAreRunningOn;
	private static String robotName;
	
	//basicAutonSelect
	static
	{
    	robotName = SmartDashboard.getString("robotName", "Mackinaw");//Default to "Mackinaw" if don't get a value
    	if("Mackinaw" == robotName)
    	{ robotWeAreRunningOn = new RobotValuesMackinac();
    	}
    	else if("LilGeek"== robotName)
    	{ 
    		robotWeAreRunningOn = new RobotValuesLilGeek();
    	}
    	else if("Rudy"== robotName){ robotWeAreRunningOn = new RobotValuesRudy();
    	}
	}

	//???
	public static boolean getRightDriveInverted(){return robotWeAreRunningOn.getRightDriveInverted();}
	public static boolean getLeftDriveInverted(){return robotWeAreRunningOn.getLeftDriveInverted();}
	
	//Controller PWM Slots
	public static int getIntakePWM(){return robotWeAreRunningOn.getIntakePWM();}//2);
	public static int getIndexerPWM(){return robotWeAreRunningOn.getIndexerPWM();}//3);
	public static int getShooterPWM(){return robotWeAreRunningOn.getShooterPWM();}
	
	//Encoder Pulses
	public static double getLeftEncoderDistancePerPulse(){return robotWeAreRunningOn.getLeftEncoderDistancePerPulse();}
	public static double getRightEncoderDistancePerPulse(){return robotWeAreRunningOn.getRightEncoderDistancePerPulse();}
	public static  double getShooterEncoderDistancePerPulse(){return robotWeAreRunningOn.getShooterEncoderDistancePerPulse();}
	
	//PID values?
	public static  double getPidDist_P(){return robotWeAreRunningOn.getPidDist_P();}
	public static  double getPidDist_I(){return robotWeAreRunningOn.getPidDist_I();}
	public static  double getPidDist_D(){return robotWeAreRunningOn.getPidDist_D();}
	public static  double getPidDist_AbsoluteTolerance(){return robotWeAreRunningOn.getPidDist_AbsoluteTolerance();}

}
