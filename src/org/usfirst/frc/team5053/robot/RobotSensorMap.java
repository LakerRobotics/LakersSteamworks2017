package org.usfirst.frc.team5053.robot;

import org.usfirst.frc.team5053.robot.sensors.LidarLiteSensor;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
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

	public static final boolean REVERSE = true;
	public static final boolean FORWARD = false;
	
	private Encoder m_LeftDrive;
	private Encoder m_RightDrive;
	private Encoder m_Shooter;
	private ADXRS450_Gyro m_Gyro;
	private LidarLiteSensor m_Lidar;
	private SerialPort m_USBLidar;
	
	private DigitalOutput m_Red;
	private DigitalOutput m_Blue;
	private DigitalOutput m_Green;

	public RobotSensorMap()
	{
		m_LeftDrive = new Encoder(leftDriveEncoderADIO, leftDriveEncoderBDIO);
		m_RightDrive = new Encoder(rightDriveEncoderADIO, rightDriveEncoderBDIO);

		m_Shooter = new Encoder(shooterEncoderADIO, shooterEncoderBDIO, FORWARD, EncodingType.k1X);// the k1X ensures we get 1024 click per revolution (think otherwise would give us 2048, 1024 from channel A and 1024 from channel B)
		
		m_LeftDrive.setDistancePerPulse(6*Math.PI/1024);//converts click to distance in inches
		m_RightDrive.setDistancePerPulse(6*Math.PI/1024);//converts click to distance in inches
		m_Shooter.setDistancePerPulse((60/1024)*(72/24));// convert 1 click/sec to RPM(rev/min): 1 Click/sec * 60sec/min * 1rev/1024clicks = rev/min
		                                                 //: 72 teeth / rev of 2nd axle * (1 rev of primary axle/24teeth)
		m_Gyro = new ADXRS450_Gyro();
		
		m_Lidar = new LidarLiteSensor();
		
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
	public Encoder getShooterEncoder() 
	{
		return m_Shooter;
	}
	public ADXRS450_Gyro getGyro() 
	{
		return m_Gyro;
	}
	public LidarLiteSensor getLidar()
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
