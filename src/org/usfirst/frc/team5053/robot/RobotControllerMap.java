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
	private final int intakePWM = 2;
	private final int indexerPWM = 3;
	private final int shooterPWM = 4;
	private final int scalerPWM = 5;
	private final int mixerPWM = 6;
	private final int gearManipulatorPWM = 7;
	
	private Talon m_LeftDrive;
	private Talon m_RightDrive;
	private Talon m_Intake;
	private Talon m_Indexer;
	private Talon m_Shooter;
	private Talon m_Scaler;
	private Talon m_Mixer;
	private Talon m_GearManipulator;
	
	/**
	 * 
	 */
	public RobotControllerMap()
	{	
		m_LeftDrive = new Talon(leftDrivePWM);
		m_RightDrive = new Talon(rightDrivePWM);
		m_Intake = new Talon(intakePWM);
		m_Indexer = new Talon(indexerPWM);
		m_Shooter = new Talon(shooterPWM);
		m_Scaler = new Talon(scalerPWM);
		m_Mixer = new Talon(mixerPWM);
		m_GearManipulator = new Talon(gearManipulatorPWM);
	}
	
	public Talon getLeftDrive()
	{
		return m_LeftDrive;
	}
	public Talon getRightDrive()
	{
		return m_RightDrive;
	}
	public Talon getGearManipulator()
	{
		return m_GearManipulator;
	}
	public Talon getShooter()
	{
		return m_Shooter;
	}
	public Talon getIntake()
	{
		return m_Intake;
	}
	public Talon getScaler() {
		return m_Scaler;
	}
	public Talon getMixer() { 
		return m_Mixer;
	}
	public Talon getIndexer() { 
		return m_Indexer;
	}
	
}