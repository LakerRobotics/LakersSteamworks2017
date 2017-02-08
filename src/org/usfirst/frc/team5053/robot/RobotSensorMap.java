package org.usfirst.frc.team5053.robot;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;

/**
 * Maps all of the sensors on the robot.
 * These include but are not limited to the following:
 * Encoders,
 * Potentiometers,
 * Ultrasonics,
 * Limit Switches,
 * Cameras
 */

//Controllers are mapped and handled by the RobotControllerMap class

public class RobotSensorMap
{
	
	private final int leftDriveEncoderADIO = 0;
	private final int leftDriveEncoderBDIO = 1;
	private final int rightDriveEncoderADIO = 2;
	private final int rightDriveEncoderBDIO = 3;
	private final int shooterEncoderADIO = 4;
	private final int shooterEncoderBDIO = 5;
	
	private Encoder m_LeftDrive;
	private Encoder m_RightDrive;
	private Encoder m_LeftShooter;
	
	private ADXRS450_Gyro m_Gyro;

	public RobotSensorMap()
	{
		m_LeftDrive = new Encoder(leftDriveEncoderADIO, leftDriveEncoderBDIO);
		m_RightDrive = new Encoder(rightDriveEncoderADIO, rightDriveEncoderBDIO);
		m_LeftShooter = new Encoder(shooterEncoderADIO, shooterEncoderBDIO);
		
		m_LeftDrive.setDistancePerPulse(6*Math.PI/360);
		m_RightDrive.setDistancePerPulse(6*Math.PI/360);
		
		m_Gyro = new ADXRS450_Gyro();
	}
	
	public Encoder getLeftDriveEncoder() 
	{
		return m_LeftDrive;
	}
	public Encoder getRightDriveEncoder() 
	{
		return m_RightDrive;
	}
	public Encoder getShooterEncoder() 
	{
		return m_LeftShooter;
	}
	public ADXRS450_Gyro getGyro() 
	{
		return m_Gyro;
	}
}
