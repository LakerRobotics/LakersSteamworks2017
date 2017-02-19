package org.usfirst.frc.team5053.robot;

import edu.wpi.first.wpilibj.PWM;
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
	private final int intakePWM = RobotConstants.getIntakePWM();//2);
	private final int indexerPWM = RobotConstants.getIndexerPWM();//3);
	private final int shooterPWM = RobotConstants.getShooterPWM();// 4 on Mackinac 2 on Rudy
	private final int scalerPWM = 5;     
//MixerDoesntExist	private final int mixerPWM = 6; // Dont think this exists
	
	private Talon m_LeftDrive;
	private Talon m_RightDrive;
	private Talon m_Intake;
	private Talon m_Indexer;
	private Talon m_Shooter;	
	private Talon m_Scaler;
	private Talon m_Mixer;
	
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
		m_LeftDrive.setInverted(RobotConstants.getLeftDriveInverted());
		m_RightDrive.setInverted(RobotConstants.getRightDriveInverted());//true
		m_Intake.setInverted(true);
		m_Shooter.setInverted(false);
	}
	
	public Talon getLeftDrive()
	{
		return m_LeftDrive;
	}
	public Talon getRightDrive()
	{
		return m_RightDrive;
	}
	public Talon getIntake()
	{
		return m_Intake;
	}
	public Talon getIndexer()
	{
		return m_Indexer;
	}
	public Talon getShooter()
	{
		return m_Shooter;
	}
	public Talon getScaler() 
	{
		return m_Scaler;
	}

}