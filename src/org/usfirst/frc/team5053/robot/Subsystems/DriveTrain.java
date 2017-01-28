package org.usfirst.frc.team5053.robot.Subsystems;

import java.util.HashMap;

import org.usfirst.frc.team5053.robot.Subsystems.Utilities.DriveStraightPIDOutput;
import org.usfirst.frc.team5053.robot.Subsystems.Utilities.MotionControlHelper;
import org.usfirst.frc.team5053.robot.Subsystems.Utilities.MotionControlPIDController;
import org.usfirst.frc.team5053.robot.Subsystems.Utilities.TurnPIDOutput;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SpeedController;
/**
 * Drivetrain subsystem that extends the FRC RobotDrive class.
 * 
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
	
	public boolean isPIDRunning = false;
	
	private MotionControlPIDController m_StraightPID;
	private MotionControlHelper m_StraightHelper;
	private final double STRAIGHT_KP = 1d/200;
	private final double STRAIGHT_KI = 0.0005;
	private final double STRAIGHT_KD = 0.0;
	
	private MotionControlPIDController m_TurnPID;
	private MotionControlHelper m_TurnHelper;
	private final double TURN_KP = 0.0;
	private final double TURN_KI = 0.0;
	private final double TURN_KD = 0.0;
	
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
	
	//Untested
	public boolean turn(double angle) {
		if(!isPIDRunning) 
		{
			double rampUpDown = 90;
			double maxSpeed = 10.0*(360/60); //10 Rotations Per Second
			
			m_TurnHelper = new MotionControlHelper(angle, rampUpDown, maxSpeed, m_Gyro.getAngle(), (PIDSource) m_Gyro, new TurnPIDOutput(this));
			m_TurnPID = new MotionControlPIDController(TURN_KP, TURN_KI, TURN_KD, m_TurnHelper);
	    	m_TurnPID.setOutputRange(-1.0, 1.0);
	    	m_TurnPID.enable();
	    	
	    	isPIDRunning = true;
		}
		else 
		{
			//TODO Tolerance?
			if(m_Gyro.getAngle() >= angle)
			{
				m_TurnPID.disable();
				isPIDRunning = false;
				
				return true;
			}
		}
		
		return false;
	}
	
	//Untested
	public boolean driveDistance(double distance) {
		if(!isPIDRunning) 
		{
			m_LeftEncoder.reset();
			m_RightEncoder.reset();
			
			double rampUpDown = 90;
			double maxSpeed = 10.0*(360/60); //10 Rotations Per Second
			
			m_StraightHelper = new MotionControlHelper(distance, rampUpDown, maxSpeed, m_LeftEncoder.getDistance(), (PIDSource) m_LeftEncoder, new DriveStraightPIDOutput(this));
			m_StraightPID = new MotionControlPIDController(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD, m_StraightHelper);
	    	m_StraightPID.setOutputRange(-1.0, 1.0);
	    	m_StraightPID.enable();
	    	
	    	isPIDRunning = true;
		}
		else 
		{
			if(m_LeftEncoder.getDistance() >= distance)
			{
				m_StraightPID.disable();
				isPIDRunning = false;
				
				return true;
			}
		}
		
		return false;
	}
	
	public HashMap<String, Double> GetDashboardData() {
		return null;
		// TODO Auto-generated method stub
		
	}
	public void WriteDashboardData() {
		// TODO Auto-generated method stub
		
	}

}
