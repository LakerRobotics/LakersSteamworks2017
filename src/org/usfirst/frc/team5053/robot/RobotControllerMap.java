package org.usfirst.frc.team5053.robot;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;

/**
 * Maps all of the output controllers on the robot.
 * These include the following:
 * Talons,
 * Victors,
 * Jaguars,
 * Spikes,
 * Solenoids
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

	private SpeedController m_LeftDrive;
	private SpeedController m_RightDrive;
	
	private SpeedController m_GearManipulator;
	private SpeedController m_Shooter;
	
	private SpeedController m_Intake;
	private SpeedController m_Indexer;
	private SpeedController m_Mixer;
	
	private SpeedController m_Scaler;

	public RobotControllerMap()
	{
		
		m_LeftDrive = new Talon(leftDrivePWM);
		m_RightDrive = new Talon(rightDrivePWM);
		
		m_GearManipulator = new Talon(gearManipulatorPWM);
		
		m_Shooter = new Talon(shooterPWM);
		
		m_Intake = new Talon(intakePWM);
		
		m_Scaler = new Talon(scalerPWM);
	}
	
	public SpeedController GetLeftDrive()
	{
		return m_LeftDrive;
	}
	public SpeedController GetRightDrive()
	{
		return m_RightDrive;
	}
	public SpeedController GetGearManipulator()
	{
		return m_GearManipulator;
	}
	public SpeedController GetShooter()
	{
		return m_Shooter;
	}
	public SpeedController GetIntake()
	{
		return m_Intake;
	}
	public SpeedController getScaler() {
		return m_Scaler;
	}
	
}