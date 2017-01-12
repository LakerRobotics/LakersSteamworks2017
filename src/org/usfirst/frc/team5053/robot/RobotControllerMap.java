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
	private final int shooterPWM = 2;
	private final int intakePWM = 3;
	private final int armPWM = 4;
	
	
	private Talon m_LeftDrive;
	private Talon m_RightDrive;
	
	private Talon m_Arm;
	private Talon m_Shooter;
	
	private Talon m_Intake;

	
	/**
	 * 
	 */
	public RobotControllerMap()
	{
		
		m_LeftDrive = new Talon(leftDrivePWM);
		m_RightDrive = new Talon(rightDrivePWM);
		
		m_Arm = new Talon(armPWM);
		
		m_Shooter = new Talon(shooterPWM);
		
		m_Intake = new Talon(intakePWM);
		
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
	public Talon GetShooter()
	{
		return m_Shooter;
	}
	public Talon GetIntake()
	{
		return m_Intake;
	}
	
}