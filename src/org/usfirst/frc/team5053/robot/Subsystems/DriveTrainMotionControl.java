package org.usfirst.frc.team5053.robot.Subsystems;

import java.util.HashMap;

import org.usfirst.frc.team5053.robot.Subsystems.Utilities.DriveStraightPIDOutput;
import org.usfirst.frc.team5053.robot.Subsystems.Utilities.MotionControlHelper;
import org.usfirst.frc.team5053.robot.Subsystems.Utilities.MotionControlPIDController;
import org.usfirst.frc.team5053.robot.Subsystems.Utilities.MotionController;
import org.usfirst.frc.team5053.robot.Subsystems.Utilities.TurnPIDOutput;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
/**
 * Drivetrain subsystem that extends the FRC RobotDrive class.
 * 
 *
 */

public class DriveTrainMotionControl extends RobotDrive implements Subsystem
{
	/**
	 * Hello There! : I'm the base constructor.
	 */
	
	private SpeedController m_LeftMotor;
	private SpeedController m_RightMotor;
	
	private Encoder m_LeftEncoder;
	private Encoder m_RightEncoder;
	
	private MotionController m_MotionController;
	
	private ADXRS450_Gyro m_Gyro;
	
	public boolean isPIDRunning = false;
	
	
	public DriveTrainMotionControl(SpeedController leftMotor, SpeedController rightMotor, Encoder leftEncoder, Encoder rightEncoder, ADXRS450_Gyro Gyro)
	{
		super(leftMotor, rightMotor);
		
		m_LeftMotor = leftMotor;
		m_RightMotor = rightMotor;
		
		m_LeftEncoder = leftEncoder;
		m_RightEncoder = rightEncoder;
		
		m_Gyro = Gyro;
		
		m_LeftMotor.setInverted(true);
		m_RightMotor.setInverted(true);
		
		m_MotionController = new MotionController(this, (PIDSource) m_RightEncoder, (PIDSource) m_Gyro);
		
	}
	public void DriveDistance(double distance, double maxspeed, double ramp)
	{
		if(!isPIDRunning)
		{
			m_MotionController.ExecuteStraightMotion(distance, maxspeed, ramp);
			isPIDRunning = true;
		}
		
	}
	public boolean isPIDFinished()
	{
		if (m_MotionController.isStraightMotionFinished())
		{
			isPIDRunning = false;
			return true;
		}
		return false;
	}
	public double GetRightDistance()
	{
		return m_RightEncoder.getDistance();
	}
	public double GetRightSpeed()
	{
		return m_RightEncoder.getRate();
	}
	public double GetLeftDistance()
	{
		return m_LeftEncoder.getDistance();
	}
	public double GetLeftSpeed()
	{
		return m_LeftEncoder.getRate();
	}
	public void ResetEncoders()
	{
		m_LeftEncoder.reset();
		m_RightEncoder.reset();
	}
	public double GetAverageSpeed()
	{
		return ((GetLeftSpeed() + GetRightSpeed())/2);
	}
	public double GetAverageDistance()
	{
		return GetLeftDistance();
	}
	public void ArcadeDrive(double speed, double angle)
	{
		this.ArcadeDrive(speed, angle);
	}
	public double GetAngle()
	{
		return m_Gyro.getAngle();
	}
	
	public HashMap<String, Double> GetDashboardData() {
		return null;
		// TODO Auto-generated method stub
		
	}
	public void WriteDashboardData() {
		// TODO Auto-generated method stub
		SmartDashboard.putNumber("Left Encoder", m_LeftEncoder.getDistance());
		SmartDashboard.putNumber("Right Encoder", m_RightEncoder.getDistance());
		SmartDashboard.putBoolean("Is DriveTrain PID Running", isPIDRunning);
	}

}
