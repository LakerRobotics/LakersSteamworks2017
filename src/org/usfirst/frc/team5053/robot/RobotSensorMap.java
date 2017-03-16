package org.usfirst.frc.team5053.robot;

import org.usfirst.frc.team5053.robot.Sensors.LidarLite;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Counter;
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
	private final int shooterEncoderADIO = 4;
	private final int shooterEncoderBDIO = 5;
	private final int redDIO = 6;
	private final int blueDIO = 7;
	private final int greenDIO = 8;
	
	private Encoder m_LeftDrive;
	private Encoder m_RightDrive;
	private Counter m_Shooter;
	
	private ADXRS450_Gyro m_Gyro;
	private LidarLite m_Lidar;
	
	private DigitalOutput m_Red;
	private DigitalOutput m_Blue;
	private DigitalOutput m_Green;

	public RobotSensorMap()
	{
		m_LeftDrive = new Encoder(leftDriveEncoderADIO, leftDriveEncoderBDIO);
		m_RightDrive = new Encoder(rightDriveEncoderADIO, rightDriveEncoderBDIO);

		//TODO SRX
		//m_Shooter = new Encoder(shooterEncoderADIO, shooterEncoderBDIO, false, EncodingType.k1X);
		m_Shooter = new Counter(shooterEncoderADIO);
		m_Shooter.setDistancePerPulse(RobotConstants.getShooterEncoderDistancePerPulse()); //(Seconds per minute/Ticks per revolution) * (72 Teeth per revolution primary/ 24 Teeth per revolution secondary)
		m_Shooter.setPIDSourceType(PIDSourceType.kRate);
		
		m_Shooter.setSamplesToAverage(60);
		m_Shooter.setMaxPeriod(0.02);
		
		m_LeftDrive.setDistancePerPulse(360/*RobotConstants.getLeftEncoderDistancePerPulse()*/); //Distance in inches
		m_Shooter.setSamplesToAverage(5);
		
		m_RightDrive.setDistancePerPulse(360/*RobotConstants.getRightEncoderDistancePerPulse()*/); //Distance in inches
		m_Shooter.setSamplesToAverage(5);
		
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
	public Counter getShooterEncoder() 
	{
		return m_Shooter;
	}
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
