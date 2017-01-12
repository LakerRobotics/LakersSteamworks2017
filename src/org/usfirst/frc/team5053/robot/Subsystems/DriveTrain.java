package org.usfirst.frc.team5053.robot.Subsystems;

import java.util.HashMap;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SpeedController;
/**
 * Drivetrain subsystem that extends the FRC RobotDrive class.
 * @author Colin Ross
 *
 */

public class DriveTrain extends RobotDrive implements Subsystem
{
	/**
	 * Hello There! : I'm the base constructor.
	 */
	
	private SpeedController m_LeftMotor;
	private SpeedController m_RightMotor;
	
	private Encoder m_LeftEncoder;
	private Encoder m_RightEncoder;
	
	private ADXRS450_Gyro m_Gyro;
	
	public DriveTrain(SpeedController leftMotor, SpeedController rightMotor) 
	{
		super(leftMotor, rightMotor);
		m_LeftMotor = leftMotor;
		m_RightMotor = rightMotor;
		
		m_LeftMotor.setInverted(true);
		m_RightMotor.setInverted(true);
	}
	public DriveTrain(SpeedController leftMotor, SpeedController rightMotor, Encoder leftEncoder, Encoder rightEncoder)
	{
		super(leftMotor, rightMotor);
		
		m_LeftMotor = leftMotor;
		m_RightMotor = rightMotor;
		
		m_LeftEncoder = leftEncoder;
		m_RightEncoder = rightEncoder;
		
		
		m_LeftMotor.setInverted(true);
		m_RightMotor.setInverted(true);
		
	}
	public DriveTrain(SpeedController leftMotor, SpeedController rightMotor, Encoder leftEncoder, Encoder rightEncoder, ADXRS450_Gyro Gyro)
	{
		super(leftMotor, rightMotor);
		
		m_LeftMotor = leftMotor;
		m_RightMotor = rightMotor;
		
		m_LeftEncoder = leftEncoder;
		m_RightEncoder = rightEncoder;
		
		m_Gyro = Gyro;
		
		m_LeftMotor.setInverted(true);
		m_RightMotor.setInverted(true);
		
	}
	public HashMap<String, Double> GetDashboardData() {
		return null;
		// TODO Auto-generated method stub
		
	}

}
