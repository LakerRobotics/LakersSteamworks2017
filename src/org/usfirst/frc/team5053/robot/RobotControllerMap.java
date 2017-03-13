package org.usfirst.frc.team5053.robot;

import org.usfirst.frc.team5053.robot.Subsystems.Utilities.TalonCurrentLimited;
import org.usfirst.frc.team5053.robot.Subsystems.Utilities.TalonVoltageStable;

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
	private final int intakePWM = RobotConstants.getIntakePWM();
	private final int indexerPWM = RobotConstants.getIndexerPWM();
	private final int shooterPWM = RobotConstants.getShooterPWM();
	private final int scalerPWM = 5;
	private final int mixerPWM = 6;
	
	private final int scalerPDPSlot = 3;
	
	private Talon m_LeftDrive;
	private Talon m_RightDrive;
	private Talon m_Intake;
	private Talon m_Indexer;
	private TalonVoltageStable m_Shooter;	
	private TalonCurrentLimited m_Scaler;
	private Talon m_Mixer;
	
	public RobotControllerMap()
	{
		
		m_LeftDrive = new Talon(leftDrivePWM);
		m_RightDrive = new Talon(rightDrivePWM);
		m_Intake = new Talon(intakePWM);
		m_Indexer = new Talon(indexerPWM);
		m_Shooter = new TalonVoltageStable(new Talon(shooterPWM));
		m_Scaler = new TalonCurrentLimited(new Talon(scalerPWM), scalerPDPSlot, 9, 9);
		m_Mixer = new Talon(mixerPWM);
		
		m_LeftDrive.setInverted(true);
		m_RightDrive.setInverted(true);
		m_Intake.setInverted(true);
		m_Shooter.setInverted(true);
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
	public TalonVoltageStable getShooter()
	{
		return m_Shooter;
	}
	public TalonCurrentLimited getScaler() 
	{
		return m_Scaler;
	}
	public Talon getMixer()
	{
		return m_Mixer;
	}
}