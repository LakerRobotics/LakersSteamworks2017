package org.usfirst.frc.team5053.robot;

import edu.wpi.first.wpilibj.Talon;

/**
 * Maps all of the output controllers on the robot.
 * These include the following:
 * Talons,
 * Victors,
 * Jaguars,
 * Spikes,
 * Solenoids
 * 
 * 
 * 
 */



//Sensors are located and handled by the RobotSensorMap class


public class RobotControllerMap
{


	private final int leftDrivePWM = 0;
	private final int rightDrivePWM = 1;
	private final int leftShooterPWM = 2;
	private final int rightShooterPWM = 3;
	private final int intakePWM = 4;
	private final int armPWM = 5;
	private final int shooterBatteryPWM = 6;
	
	
	private Talon m_LeftDrive;
	private Talon m_RightDrive;
	
	private Talon m_Arm;
	
	private Talon m_LeftShooter;
	private Talon m_RightShooter;
	
	private Talon m_Intake;
	
	private Talon m_ShooterBattery;

	
	/**
	 * 
	 */
	public RobotControllerMap()
	{
		
		m_LeftDrive = new Talon(leftDrivePWM);
		m_RightDrive = new Talon(rightDrivePWM);
		
		m_Arm = new Talon(armPWM);
		
		m_LeftShooter = new Talon(leftShooterPWM);
		m_RightShooter = new Talon(rightShooterPWM);
		
		m_Intake = new Talon(intakePWM);
		
		m_ShooterBattery = new Talon(shooterBatteryPWM);
		
	}
	
	public Talon GetLeftDrive()
	{
		return m_LeftDrive;
	}
	public Talon GetRightDrive()
	{
		return m_RightDrive;
	}
	public Talon GetArm()
	{
		return m_Arm;
	}
	public Talon GetLeftShooter()
	{
		return m_LeftShooter;
	}
	public Talon GetRightShooter()
	{
		return m_RightShooter;
	}
	public Talon GetIntake()
	{
		return m_Intake;
	}
	public Talon GetShooterBattery() 
	{
		return m_ShooterBattery;
	}
	
}