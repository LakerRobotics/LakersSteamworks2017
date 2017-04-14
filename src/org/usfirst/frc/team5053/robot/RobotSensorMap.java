package org.usfirst.frc.team5053.robot;

import org.usfirst.frc.team5053.robot.Sensors.LidarLite;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DigitalOutput;

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
//	private final int shooterEncoderADIO = 4;
//	private final int shooterEncoderBDIO = 5;
	private final int redDIO = 6;
	private final int blueDIO = 7;
	private final int greenDIO = 8;
	
	private Encoder m_LeftDrive;
	private Encoder m_RightDrive;
//	private Encoder m_Shooter; if using talon then encoder does not talk to RoboRio so dont define it here, gets configured and defined in TalonSRX motor controller
	
	private ADXRS450_Gyro m_Gyro;
	private LidarLite m_Lidar;
	
	private DigitalOutput m_Red;
	private DigitalOutput m_Blue;
	private DigitalOutput m_Green;

	public RobotSensorMap()
	{
		m_LeftDrive = new Encoder(leftDriveEncoderADIO, leftDriveEncoderBDIO);
		m_LeftDrive.setReverseDirection(true);
		m_RightDrive = new Encoder(rightDriveEncoderADIO, rightDriveEncoderBDIO);
		m_RightDrive.setReverseDirection(true);
//		m_Shooter = new Encoder(shooterEncoderADIO, shooterEncoderBDIO, false, EncodingType.k1X);
//		m_Shooter.setSamplesToAverage(60);
//		m_Shooter.setMaxPeriod(0.02);
		
		//TODO 360 -> 1024
		m_LeftDrive.setDistancePerPulse(6*Math.PI/1024); //Distance in inches
		m_RightDrive.setDistancePerPulse(6*Math.PI/1024); //Distance in inches
//		m_Shooter.setDistancePerPulse(/*60/20*/(60.0d/1024.0d)*(72.0d/24.0d)); //(Seconds per minute/Ticks per revolution) * (72 Teeth per revolution primary/ 24 Teeth per revolution secondary)
//		m_Shooter.setPIDSourceType(PIDSourceType.kRate);
		
		m_Gyro = new ADXRS450_Gyro();
		m_Lidar = new LidarLite();
		
		m_Red = new DigitalOutput(redDIO);
		m_Blue = new DigitalOutput(blueDIO);
		m_Green = new DigitalOutput(greenDIO);
	}
	
	public Encoder getLeftDriveEncoder() 
	{
		return m_LeftDrive;
	}
	public Encoder getRightDriveEncoder() 
	{
		return m_RightDrive;
	}
//	public Encoder getShooterEncoder() 
//	{
//		return m_Shooter;
//	}
	public ADXRS450_Gyro getGyro() 
	{
		return m_Gyro;
	}
	public LidarLite getLidar()
	{
		return m_Lidar;
	}
	public DigitalOutput getRed()
	{
		return m_Red;
	}
	public DigitalOutput getBlue()
	{
		return m_Blue;
	}
	public DigitalOutput getGreen()
	{
		return m_Green;
	}
}
