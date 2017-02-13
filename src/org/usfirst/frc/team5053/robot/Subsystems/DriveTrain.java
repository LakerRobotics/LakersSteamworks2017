package org.usfirst.frc.team5053.robot.Subsystems;

import java.util.HashMap;

import org.usfirst.frc.team5053.robot.Subsystems.Utilities.AnglePIDWrapper;
import org.usfirst.frc.team5053.robot.Subsystems.Utilities.DistancePIDWrapper;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
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
	
	private PIDController m_DistancePID;
	private PIDController m_AnglePID;
	
	private DistancePIDWrapper m_DistancePIDWrapper;
	private AnglePIDWrapper m_AnglePIDWrapper;
	
	
	private Encoder m_LeftEncoder;
	private Encoder m_RightEncoder;
	
	private ADXRS450_Gyro m_Gyro;
	
	private double m_speed = 0.0;
	private double m_turn = 0.0;
	
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
		
		m_DistancePIDWrapper = new DistancePIDWrapper(this);
		
		
		this.setExpiration(0.1);
		
		
		m_DistancePID = new PIDController(0.1, 0.0, 0.0, m_DistancePIDWrapper, m_DistancePIDWrapper);
		m_DistancePID.setAbsoluteTolerance(5.2);
		
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
		
		m_DistancePIDWrapper = new DistancePIDWrapper(this);
		m_AnglePIDWrapper = new AnglePIDWrapper(this);
		
		
		//this.setExpiration(0.1);
		
		
		m_DistancePID = new PIDController(0.1, 0.0, 0.0, m_DistancePIDWrapper, m_DistancePIDWrapper);
		m_DistancePID.setAbsoluteTolerance(5.2);
		
		m_AnglePID = new PIDController(0.1, 0.0, 0.0, m_AnglePIDWrapper, m_AnglePIDWrapper);
		m_AnglePID.setAbsoluteTolerance(2.5);
		
		System.out.println("Constructor finished");
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
	public double GetPIDSetpoint()
	{
		return m_DistancePIDWrapper.pidGet();
	}
	
	public void ArcadeDrive(double speed, double angle)
	{
		m_speed = speed;
		m_turn = angle;
		this.ArcadeDrive(speed, angle);
	}
	public void SetSpeed(double speed)
	{
		this.ArcadeDrive(speed, m_turn);
	}
	public double GetAngle()
	{
		return m_Gyro.getAngle();
	}
	public void ResetAngle()
	{
		m_Gyro.reset();
	}
	public double GetAngularVelocity()
	{
		return m_Gyro.getRate();
	}
	public void SetTurn(double turn)
	{
		ArcadeDrive(m_speed, turn);
	}
	public double GetSpeed()
	{
		return m_speed;
	}
	public double GetTurn()
	{
		return m_turn;
	}
	public void SetDistancePIDMax(double maximum)
	{
		m_DistancePID.setOutputRange(-maximum, maximum);
	}
	public void SetAnglePID(float p, float i, float d)
	{
		m_AnglePID.setPID(p, i, d);
	}
	
	public void EnablePID() 
	{
		if (!m_DistancePID.isEnabled())
			m_DistancePID.enable();
		
		if (!m_AnglePID.isEnabled()) {
			m_AnglePID.enable();
		}
	}
	public void DisablePID()
	{
		if (m_DistancePID.isEnabled())
			m_DistancePID.disable();
		if (m_AnglePID.isEnabled()) {
			m_AnglePID.disable();
		}
	}
	public boolean DistanceOnTarget()
	{
		if (Math.abs(GetDistancePIDSetpoint() - GetAverageDistance()) < 4)
			return true;
		else return false;
	}
	public boolean AngleOnTarget()
	{
		if(Math.abs(GetAnglePIDSetpoint() - GetAngle()) < 2.5)
		{
			return true;
		} else return false;
	}
	public void SetPIDSetpoint(double distance, double angle)
	{
		m_DistancePID.setSetpoint(distance);
		m_AnglePID.setSetpoint(angle);
		
	}
	double GetDistancePIDSetpoint() 
	{
		return m_DistancePID.getSetpoint();
	}
	double GetAnglePIDSetpoint()
	{
		return m_AnglePID.getSetpoint();
	}
	public HashMap<String, Double> GetDashboardData() {
		return null;
		// TODO Auto-generated method stub
		
	}
	@Override
	public void WriteDashboardData() {
		// TODO Auto-generated method stub
		
	}

}
