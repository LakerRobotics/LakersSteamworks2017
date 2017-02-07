package org.usfirst.frc.team5053.robot.Subsystems;

import java.util.HashMap;

import org.usfirst.frc.team5053.robot.Subsystems.Utilities.AnglePIDWrapper;
import org.usfirst.frc.team5053.robot.Subsystems.Utilities.CurvePIDWrapper;
import org.usfirst.frc.team5053.robot.Subsystems.Utilities.DistancePIDWrapper;

import java.lang.*;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
/**
 * Drivetrain subsystem that extends the FRC RobotDrive class.
 * @author Colin Ross
 *
 */

public class DriveTrain implements Subsystem
{
	
	private SpeedController m_LeftMotor;
	private SpeedController m_RightMotor;
	
	private PIDController m_DistancePID;
	private PIDController m_AnglePID;
	private PIDController m_CurvePID;
	
	private DistancePIDWrapper m_DistancePIDWrapper;
	private AnglePIDWrapper m_AnglePIDWrapper;
	private CurvePIDWrapper m_CurvePIDWrapper;
	
	private Encoder m_LeftEncoder;
	private Encoder m_RightEncoder;
	
	private ADXRS450_Gyro m_Gyro;
	
	private double m_speed = 0.0;
	private double m_turn = 0.0;
	private boolean m_isLeftTurn = false;
	
	private RobotDrive m_RobotDrive;
	
	private final double STRAIGHT_KP = 0.1;
	private final double STRAIGHT_KI = 0.0;
	private final double STRAIGHT_KD = 0.0;
	
	private final double TURN_KP = 0.9;
	private final double TURN_KI = 0.0;
	private final double TURN_KD = 0.0;
	
	private final double CURVE_KP = 0.0;
	private final double CURVE_KI = 0.0;
	private final double CURVE_KD = 0.0;
	
	public DriveTrain(SpeedController leftMotor, SpeedController rightMotor) 
	{
		m_RobotDrive = new RobotDrive(leftMotor, rightMotor);
		
		m_LeftMotor = leftMotor;
		m_RightMotor = rightMotor;
		
		m_LeftMotor.setInverted(true);
		m_RightMotor.setInverted(true);
		
		
	}
	public DriveTrain(SpeedController leftMotor, SpeedController rightMotor, Encoder leftEncoder, Encoder rightEncoder)
	{
		m_RobotDrive = new RobotDrive(leftMotor, rightMotor);
		
		m_LeftMotor = leftMotor;
		m_RightMotor = rightMotor;
		
		m_LeftEncoder = leftEncoder;
		m_RightEncoder = rightEncoder;
		
		
		m_LeftMotor.setInverted(true);
		m_RightMotor.setInverted(true);
		
		m_DistancePIDWrapper = new DistancePIDWrapper(this);
		
		m_DistancePID = new PIDController(0.1, 0.0, 0.0, m_DistancePIDWrapper, m_DistancePIDWrapper);
		m_DistancePID.setAbsoluteTolerance(5.2);
		
	}
	public DriveTrain(SpeedController leftMotor, SpeedController rightMotor, Encoder leftEncoder, Encoder rightEncoder, ADXRS450_Gyro Gyro)
	{
		m_RobotDrive = new RobotDrive(leftMotor, rightMotor);
		
		m_LeftMotor = leftMotor;
		m_RightMotor = rightMotor;
		
		m_LeftEncoder = leftEncoder;
		m_RightEncoder = rightEncoder;
		
		m_LeftEncoder.setReverseDirection(true);
		m_RightEncoder.setReverseDirection(true);
		
		m_Gyro = Gyro;
		
		m_LeftMotor.setInverted(true);
		m_RightMotor.setInverted(true);
		
		m_DistancePIDWrapper = new DistancePIDWrapper(this);
		m_AnglePIDWrapper = new AnglePIDWrapper(this);
		m_CurvePIDWrapper = new CurvePIDWrapper(this);

		m_DistancePID = new PIDController(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD, m_DistancePIDWrapper, m_DistancePIDWrapper);
		m_DistancePID.setAbsoluteTolerance(5.2);
		
		m_AnglePID = new PIDController(TURN_KP, TURN_KI, TURN_KD, m_AnglePIDWrapper, m_AnglePIDWrapper);
		m_AnglePID.setAbsoluteTolerance(2.5);
		
		m_CurvePID = new PIDController(CURVE_KP, CURVE_KI, CURVE_KD, m_CurvePIDWrapper, m_CurvePIDWrapper);
		
		
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
		return (GetLeftDistance() + GetRightDistance())/2;
		//return GetLeftDistance();
	}
	
	public double GetPIDSetpoint()
	{
		return m_DistancePIDWrapper.pidGet();
	}
	public double GetCurveSetpoint()
	{
		return m_CurvePIDWrapper.pidGet();
	}
	public void ArcadeDrive(double speed, double angle)
	{
		m_speed = speed;
		m_turn = angle;
		m_RobotDrive.arcadeDrive(speed, angle);
	}
	public void TankDrive(double speed, boolean isLeftTurn)
	{
		if(isLeftTurn)
		{
			m_RobotDrive.tankDrive(0, speed);
		}
		else
		{
			m_RobotDrive.tankDrive(speed, 0);
		}
		m_isLeftTurn = isLeftTurn;
	}
	public void TeleopDrive(double speed, double angle)
	{
		m_RobotDrive.arcadeDrive(speed, angle);
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
		if(m_CurvePID.isEnabled())
		{
			m_CurvePID.disable();
		}
		if (!m_DistancePID.isEnabled())
		{
			m_DistancePID.enable();
		}
		if (!m_AnglePID.isEnabled()) 
		{
			m_AnglePID.enable();
		}
	}
	public void EnableCurvePID() {
		if(m_DistancePID.isEnabled())
		{
			m_DistancePID.disable();
		}
		if(m_AnglePID.isEnabled())
		{
			m_AnglePID.disable();
		}
		if(!m_CurvePID.isEnabled())
		{
			m_CurvePID.enable();
		}
	}
	public void DisablePID()
	{
		if (m_DistancePID.isEnabled())
		{
			m_DistancePID.disable();
		}
		if (m_AnglePID.isEnabled()) 
		{
			m_AnglePID.disable();
		}
		if(m_CurvePID.isEnabled())
		{
			m_CurvePID.disable();
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
	public boolean CurveOnTarget()
	{
		if(Math.abs(getCurvePIDSetpoint() - GetAngle()) < 2.5)
		{
			return true;
		}
		else
			return false;
	}
	public void SetPIDSetpoint(double distance, double angle)
	{
		m_DistancePID.setSetpoint(distance);
		m_AnglePID.setSetpoint(angle);	
	}
	public void SetCurveSetpoint(double angle)
	{
		m_CurvePID.setSetpoint(angle);
	}
	double GetDistancePIDSetpoint() 
	{
		return m_DistancePID.getSetpoint();
	}
	double GetAnglePIDSetpoint()
	{
		return m_AnglePID.getSetpoint();
	}
	double getCurvePIDSetpoint()
	{
		return m_CurvePID.getSetpoint();
	}
	public HashMap<String, Double> GetDashboardData() {
		return null;
	}
	@Override
	public void WriteDashboardData() {
		SmartDashboard.putNumber("Encoder Distance", GetAverageDistance());
		SmartDashboard.putNumber("Gyro Angle", GetAngle());
		SmartDashboard.putNumber("Curve output", GetCurveSetpoint());
	}
	public void SetCurve(double speed) {
		TankDrive(speed, m_isLeftTurn);
	}

}
