package org.usfirst.frc.team5053.robot.Subsystems;

import java.util.HashMap;

import org.usfirst.frc.team5053.robot.Subsystems.Utilities.DriveStraightPIDOutput;
import org.usfirst.frc.team5053.robot.Subsystems.Utilities.MotionControlHelper;
import org.usfirst.frc.team5053.robot.Subsystems.Utilities.MotionControlPIDController;
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
	
	private ADXRS450_Gyro m_Gyro;
	
	public boolean isPIDRunning = false;
	
	private MotionControlPIDController m_StraightPID;
	private MotionControlHelper m_StraightHelper;
	private final double STRAIGHT_KP = 0.005;
	private final double STRAIGHT_KI = 0.0;
	private final double STRAIGHT_KD = 0.0;
	
	private MotionControlPIDController m_TurnPID;
	private MotionControlHelper m_TurnHelper;
	private final double TURN_KP = 0.0;
	private final double TURN_KI = 0.0;
	private final double TURN_KD = 0.0;
	private final double TURN_TOLERANCE = 3;
	
	public DriveTrainMotionControl(SpeedController leftMotor, SpeedController rightMotor) 
	{
		super(leftMotor, rightMotor);
		m_LeftMotor = leftMotor;
		m_RightMotor = rightMotor;
		
		m_LeftMotor.setInverted(true);
		m_RightMotor.setInverted(true);
	}
	public DriveTrainMotionControl(SpeedController leftMotor, SpeedController rightMotor, Encoder leftEncoder, Encoder rightEncoder)
	{
		super(leftMotor, rightMotor);
		
		m_LeftMotor = leftMotor;
		m_RightMotor = rightMotor;
		
		m_LeftEncoder = leftEncoder;
		m_RightEncoder = rightEncoder;
		
		
		m_LeftMotor.setInverted(true);
		m_RightMotor.setInverted(true);
		
	}
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
		
	}
	
	//Untested
	public boolean turn(double angle) throws NullPointerException {
		if(!isPIDRunning) 
		{
			m_Gyro.reset();
			m_Gyro.setPIDSourceType(PIDSourceType.kRate);
			
			double rampUpDown = 10;
			double maxSpeed = 10.0*(360/60); //10 Rotations Per Second
			
			/*m_TurnHelper = new MotionControlHelper(angle, rampUpDown, maxSpeed, m_Gyro.getAngle(), (PIDSource) m_Gyro,  new TurnPIDOutput(this)));
			m_TurnPID = new MotionControlPIDController(TURN_KP, TURN_KI, TURN_KD, m_TurnHelper);
	    	m_TurnPID.setOutputRange(-1.0, 1.0);
	    	m_TurnPID.enable();*/
	    	
	    	isPIDRunning = true;
		}
		else 
		{
			if(Math.abs(m_Gyro.getAngle()) <= Math.abs(angle) + TURN_TOLERANCE && Math.abs(m_Gyro.getAngle()) >= Math.abs(angle)  - TURN_TOLERANCE)
			{
				m_TurnPID.disable();
				isPIDRunning = false;
				
				return true;
			}
		}
		
		return false;
	}
	
	//Untested
	public boolean driveDistance(double distance) throws NullPointerException {
		if(!isPIDRunning) 
		{
			m_LeftEncoder.reset();
			m_LeftEncoder.setPIDSourceType(PIDSourceType.kRate);
			m_RightEncoder.reset();
			m_RightEncoder.setPIDSourceType(PIDSourceType.kRate);
			
			double rampUpDown = 7;
			double maxSpeed = 12.0; //12 Rotations Per Second
			
			/*m_StraightHelper = new MotionControlHelper(distance, rampUpDown, maxSpeed, m_LeftEncoder.getDistance(), (PIDSource) m_LeftEncoder, new DriveStraightPIDOutput(this));
			m_StraightPID = new MotionControlPIDController(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD, m_StraightHelper);
			
			m_StraightPID.setOutputRange(-1.0, 1.0);
	    	m_StraightPID.enable();*/
	    	
	    	isPIDRunning = true;
		}
		else 
		{
			if(Math.abs(m_LeftEncoder.getDistance()) >= Math.abs(distance))
			{
				m_StraightPID.disable();
				isPIDRunning = false;
				
				return true;
			}
		}
		
		return false;
	}
	
	private double getDistance() throws NullPointerException {
		return (m_RightEncoder.getDistance() + m_LeftEncoder.getDistance())/2;
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
