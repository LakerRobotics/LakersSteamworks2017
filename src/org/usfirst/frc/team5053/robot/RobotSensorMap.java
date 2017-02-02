package org.usfirst.frc.team5053.robot;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Solenoid;

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
	private final int gearManipulatorEncoderADIO = 6;
	private final int gearManipulatorEncoderBDIO = 7;
	
	private Encoder m_LeftDrive;
	private Encoder m_RightDrive;
	private Encoder m_LeftShooter;
	private Encoder m_GearManipulator;
	
	private AnalogPotentiometer m_Encoder;
	private ADXRS450_Gyro m_Gyro;
	

	public RobotSensorMap()
	{
		m_LeftDrive = new Encoder(leftDriveEncoderADIO, leftDriveEncoderBDIO);
		m_RightDrive = new Encoder(rightDriveEncoderADIO, rightDriveEncoderBDIO);
		m_LeftShooter = new Encoder(shooterEncoderADIO, shooterEncoderBDIO);
		m_GearManipulator = new Encoder(gearManipulatorEncoderADIO, gearManipulatorEncoderBDIO);
		
		m_Gyro = new ADXRS450_Gyro();
	}
	
	public Encoder GetLeftDriveEncoder() {
		return m_LeftDrive;
	}
	public Encoder GetRightDriveEncoder() {
		return m_RightDrive;
	}
	public Encoder GetShooterEncoder() {
		return m_LeftShooter;
	}
	public Encoder GetGearManipulator() {
		return m_GearManipulator;
	}
	public ADXRS450_Gyro GetGyro() {
		return m_Gyro;
	}
	/*
	public double getLeftEncoderRate()
	{
		if (m_LeftEncoder != null)
		{
			return m_LeftEncoder.getRate();
		}
		else 
		{
			return 0.0;
		}
	}
	public double getRightEncoderRate()
	{
		if (m_RightEncoder != null)
		{
			return m_RightEncoder.getRate();
		}
		else
		{
			return 0.0;
		}
	}
	
	public double getLeftEncoderDistance()
	{
		if (m_LeftEncoder != null)
		{
			return m_LeftEncoder.getDistance();
		}
		else 
		{
			return 0.0;
		}
	}
	public double getRightEncoderDistance()
	{
		if (m_RightEncoder != null)
		{
			return m_RightEncoder.getDistance();
		}
		else 
		{
			return 0.0;
		}
	}
*/

}
