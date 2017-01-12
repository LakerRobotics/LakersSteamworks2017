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
	private final int leftShooterEncoderADIO = 4;
	private final int leftShooterEncoderBDIO = 5;
	private final int rightShooterEncoderADIO = 6;
	private final int rightShooterEncoderBDIO = 7;
	private final int shooterLimitSwitchHighDIO = 8;
	private final int shooterLimitSwitchLowDIO = 9;
	
	private final int solenoidKicker = 0;
	private final int armStringPotentiometerAnalog = 1;
	
	private Encoder m_LeftDrive;
	private Encoder m_RightDrive;
	private Encoder m_LeftShooter;
	private Encoder m_RightShooter;
	private DigitalInput m_ShooterHigh;
	private DigitalInput m_ShooterLow;
	
	private Solenoid m_Kicker;
	private AnalogPotentiometer m_ArmPot;
	private ADXRS450_Gyro m_Gyro;
	
	/**
<<<<<<< HEAD
	 
=======
	 
>>>>>>> refs/remotes/origin/master
	 */
	public RobotSensorMap()
	{
		m_LeftDrive = new Encoder(leftDriveEncoderADIO, leftDriveEncoderBDIO);
		m_RightDrive = new Encoder(rightDriveEncoderADIO, rightDriveEncoderBDIO);
		m_LeftShooter = new Encoder(leftShooterEncoderADIO, leftShooterEncoderBDIO);
		m_RightShooter = new Encoder(rightShooterEncoderADIO, rightShooterEncoderBDIO);
		
		m_ShooterHigh = new DigitalInput(shooterLimitSwitchHighDIO);
		m_ShooterLow = new DigitalInput(shooterLimitSwitchLowDIO);
		
		m_Kicker = new Solenoid(solenoidKicker);
		m_ArmPot = new AnalogPotentiometer(armStringPotentiometerAnalog);
		m_Gyro = new ADXRS450_Gyro();
	}
	
	public Encoder GetLeftDriveEncoder() {
		return m_LeftDrive;
	}
	public Encoder GetRightDriveEncoder() {
		return m_RightDrive;
		}
	public Encoder GetLeftShooterEncoder() {
		return m_LeftShooter;
	}
	public Encoder GetRightShooterEncoder() {
		return m_RightShooter;
	}
	public DigitalInput GetShooterHigh() {
		return m_ShooterHigh;
	}
	public DigitalInput GetShooterLow() {
		return m_ShooterLow;
	}
	public Solenoid GetKickerSolenoid() {
		return m_Kicker;
	}
	public AnalogPotentiometer GetArmPot() {
		return m_ArmPot;
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
