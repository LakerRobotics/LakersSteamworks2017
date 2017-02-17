package org.usfirst.frc.team5053.robot;

import org.usfirst.frc.team5053.robot.constants.RobotValues;
import org.usfirst.frc.team5053.robot.constants.RobotValuesMackinac;
import org.usfirst.frc.team5053.robot.constants.RobotValuesLilGeek;
import org.usfirst.frc.team5053.robot.constants.RobotValuesRudy;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public  class RobotConstants extends RobotValues {
	private static RobotValues robotWeAreRunningOn;
	private static String robotName;
	
	static{
    	robotName = SmartDashboard.getString("robotName","Rudy");//Default to "Mackinaw" if don't get a value
    	if("Mackinaw"==robotName){ robotWeAreRunningOn = new RobotValuesMackinac();};
    	if("LilGeek"==robotName){ robotWeAreRunningOn = new RobotValuesLilGeek();};
    	if("Rudy"==robotName){ robotWeAreRunningOn = new RobotValuesRudy();};
	}
	@Override
	public  int getIntakePWM(){return robotWeAreRunningOn.getIntakePWM();};//2);
	@Override
	public  int getIndexerPWM(){return robotWeAreRunningOn.getIndexerPWM();};//3);
	@Override
	public  int getShooterPWM(){return robotWeAreRunningOn.getShooterPWM();};
	
	@Override
	public  double getLeftEncoderDistancePerPulse(){return robotWeAreRunningOn.getLeftEncoderDistancePerPulse();};
	@Override
	public  double getRightEncoderDistancePerPulse(){return robotWeAreRunningOn.getRightEncoderDistancePerPulse();};
	@Override
	public  double getShooterEncoderDistancePerPulse(){return robotWeAreRunningOn.getShooterEncoderDistancePerPulse();};
	
	@Override
	public  double getPidDist_P(){return robotWeAreRunningOn.getPidDist_P();};
	@Override
	public  double getPidDist_I(){return robotWeAreRunningOn.getPidDist_I();};
	@Override
	public  double getPidDist_D(){return robotWeAreRunningOn.getPidDist_D();};
	@Override
	public  double getPidDist_AbsoluteTolerance(){return robotWeAreRunningOn.getPidDist_AbsoluteTolerance();};

}
