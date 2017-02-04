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
	private final int gearManipulatorPWM = 4;
	private final int scalerPWM = 5;
	
	
	private Talon m_LeftDrive;
	private Talon m_RightDrive;
	
	private Talon m_GearManipulator;
	private Talon m_Shooter;
	
	private Talon m_Intake;
	
	private Talon m_Scaler;

	
	/**
	 * 
	 */
	public RobotControllerMap()
	{
		
		m_LeftDrive = new Talon(leftDrivePWM);
		m_RightDrive = new Talon(rightDrivePWM);
		
		m_GearManipulator = new Talon(gearManipulatorPWM);
		
		m_Shooter = new Talon(shooterPWM);
		
		m_Intake = new Talon(intakePWM);
		
		m_Scaler = new Talon(scalerPWM);
	}
	
	public Talon GetLeftDrive()
	{
		return m_LeftDrive;
	}
	public Talon GetRightDrive()
	{
		return m_RightDrive;
	}
	public Talon GetGearManipulator()
	{
		return m_GearManipulator;
	}
	public Talon GetShooter()
	{
		return m_Shooter;
	}
	public Talon GetIntake()
	{
		return m_Intake;
	}
	public Talon getScaler() {
		return m_Scaler;
	}
	
}