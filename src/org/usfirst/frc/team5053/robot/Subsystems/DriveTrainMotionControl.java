package org.usfirst.frc.team5053.robot.Subsystems;

import java.util.HashMap;

import org.usfirst.frc.team5053.robot.RobotConstants;
import org.usfirst.frc.team5053.robot.Subsystems.Utilities.MotionController;
import org.usfirst.frc.team5053.robot.Subsystems.Utilities.WrapDriveTrainAvgSpeedPIDSource;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDSource;
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
		
		WrapDriveTrainAvgSpeedPIDSource avgSpeed = new WrapDriveTrainAvgSpeedPIDSource(this);
		
		m_MotionController = new MotionController(this, (PIDSource)avgSpeed , (PIDSource) m_Gyro);
		
	}
	
	/**
	 * This will start driving the specified distance, if one is not running, 
	 * or return "false" if already running and "true" when complete
	 * @param distance
	 * @param maxspeed
	 * @param ramp
	 * @return if the Drive Distance is complete.
	 */
	public boolean DriveDistance(double distance, double maxspeed, double ramp)
	{
		System.out.print("DriveTrainMotionControl.DriveDistance isPIDRunning="+isPIDRunning);
		return 	m_MotionController.ExecuteStraightMotion(distance, maxspeed, ramp);
	}
	
	/**
	 * This will start a turn angle (if one is not running) and report false that the turn is not yet complete
	 *  or if running
	 *    Will report false if the turn is not yet complete
	 *    Will report true if the turn is done
	 * @param turnAngle
	 * @return if the turn angle is completed 
	 */
	public boolean TurnToAngle(double turnAngle)
	{
		 return m_MotionController.ExecuteTurnMotion(turnAngle);
	}
	public boolean alignToAngle(double turnAngle, Joystick driverJoyStick, int forwardPowerAxis)
	{
		return m_MotionController.ExecuteAlignMotion(turnAngle, driverJoyStick, forwardPowerAxis);
	}
	
	public void DisablePIDControlsAll()
	{
		m_MotionController.DisablePIDControlsAll();
	}

	public void DisablePIDControlTurn()
	{
		m_MotionController.DisablePIDControlsTurn();
	}

	public void DisablePIDControlStraight()
	{
		m_MotionController.DisablePIDControlsStraight();
	}
	
	public void DisablePIDControlArc()
	{
		m_MotionController.DisablePIDControlsArch();
	}
	
	public void DisablePIDControlJoystickGyro()
	{
		m_MotionController.DisablePIDControlsJoystickGyroAssist();
	}
	
	public double GetRightDistance(){	return m_RightEncoder.getDistance();	}
	public double GetRightSpeed(){	    return m_RightEncoder.getRate();}
	
	public double GetLeftDistance()	{   return m_LeftEncoder.getDistance(); }
	public double GetLeftSpeed(){       return m_LeftEncoder.getRate();}
	
	public double GetAverageSpeed(){    return ((GetLeftSpeed()   +GetRightSpeed()   )/2);}
	public double GetAverageDistance(){	return ((GetLeftDistance()+GetRightDistance())/2);}
	
	public void ResetEncoders()
	{
		m_LeftEncoder.reset();
		m_RightEncoder.reset();
	}
	public void ResetGyro() 
	{
		m_Gyro.reset();
	}
	
	public void ArcadeDrive(double speed, double angle)
	{
		this.arcadeDrive(speed, angle);
	}
	
	public void ArcadeDriveGyroAssist(Joystick joystick, int forwardPowerAxis, int rotationSpeedAxis ) 
	{
		m_MotionController.doFowardPower_and_RotationSpeedMotion(joystick,  forwardPowerAxis, rotationSpeedAxis );
	}
	
	public double GetAngle(){return m_Gyro.getAngle();}
	
	public HashMap<String, Double> GetDashboardData() 
	{
		return null;
		// TODO Auto-generated method stub
	}
	
	public void WriteDashboardData() 
	{
		SmartDashboard.putNumber("Left Encoder", m_LeftEncoder.getDistance());
		SmartDashboard.putNumber("Right Encoder", m_RightEncoder.getDistance());
		SmartDashboard.putBoolean("Is DriveTrain PID Running", isPIDRunning);
	}

}
