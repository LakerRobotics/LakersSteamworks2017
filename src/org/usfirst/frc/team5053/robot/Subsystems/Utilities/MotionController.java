package org.usfirst.frc.team5053.robot.Subsystems.Utilities;

import org.usfirst.frc.team5053.robot.Subsystems.DriveTrainMotionControl;
import org.usfirst.frc.team5053.robot.Subsystems.Utilities.MotionControlHelper;
import org.usfirst.frc.team5053.robot.Subsystems.Utilities.MotionControlPIDController;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class MotionController {
	DriveTrainMotionControl m_DriveTrain;
	MotionControlHelper m_StraightControl;
	MotionControlHelper m_TurnControl;
	
	MotionControlPIDController m_StraightPIDController;
	StraightMotionPIDOutput m_StraightPIDOutput;
	MotionControlPIDController m_TurnPIDController;
	
	private double m_targetDistance;
	private double m_targetAngle;
	private double m_straightTolerance;
	private double m_turnTolerance;
	private boolean m_PIDEnabled;
	
	private final double TurnKp = 0.9;
	private final double TurnKi = 0.0;
	private final double TurnKd = 0.0;
	private final double StraightKp = 0.005;
	private final double StraightKi = 0.0;
	private final double StraightKd = 0.0;
	
	PIDSource m_StraightSource;
	PIDSource m_TurnSource;
	
	
	
	public MotionController(DriveTrainMotionControl driveTrainMotionControl, PIDSource straightSource, PIDSource turnSource)
	{
		m_DriveTrain = driveTrainMotionControl;
		m_StraightSource = straightSource;
		m_TurnSource = turnSource;
		
		m_StraightPIDController = null;
		m_StraightPIDOutput = null;
		m_TurnPIDController = null;
		
		
		m_targetDistance = 0;
		m_targetAngle = 0;
		m_straightTolerance = 10;
		m_turnTolerance = 2;
		m_PIDEnabled = false;
		
	}
	public boolean ExecuteStraightMotion(double distance, double maxspeed, double ramp)
	{
		if (!m_PIDEnabled)
		{
			m_targetAngle = m_DriveTrain.GetAngle();
			m_targetDistance = distance;
			m_DriveTrain.ResetEncoders();
			
			double start = 0;
			
			double convertedDistance = distance;
			double convertedSpeed = maxspeed * 12; // Inches
			double convertedRamp = ramp;
			
			if (!(Math.abs(m_DriveTrain.GetLeftDistance()) > Math.abs(m_targetDistance)))
			{
				//Instantiates a new MotionControlHelper() object for the new drive segment
				m_StraightPIDOutput = new StraightMotionPIDOutput(m_DriveTrain, m_TurnSource, m_targetAngle);
				m_StraightControl = new MotionControlHelper(convertedDistance, convertedRamp, convertedSpeed, start, m_StraightSource, m_StraightPIDOutput);
				
				//Instantiates a new MotionControlPIDController() object for the new drive segment using the previous MotionControlHelper()
				m_StraightPIDController = new MotionControlPIDController(StraightKp, StraightKi, StraightKd, m_StraightControl);
				m_StraightPIDController.setAbsoluteTolerance(m_straightTolerance);
				m_StraightPIDController.setOutputRange(-1.0, 1.0);
				
				//Turns the MotionControlPID ON and it will continue to execute by itself until told otherwise.
				m_StraightPIDController.enable();
				m_PIDEnabled = true;
				return true;
			}
			return false;
		}
		return true;
	}
	public boolean ExecuteTurnMotion(double turnAngle)
	{
		if (!m_PIDEnabled)
		{
			m_DriveTrain.ResetGyro();
			
			//Magic numbers need fixing
			double maxRPM = 60/*30*/;
			double ramp = 50/* 3.5 * maxRPM*/;
			
			double maxSpeed = maxRPM * 6; //360 Degrees/60 seconds to convert RPM to speed or degrees per second
			double start = m_DriveTrain.GetAngle();
			m_targetAngle = turnAngle + start;
			
			if (!(Math.abs(m_DriveTrain.GetAngle()-m_targetAngle) < m_turnTolerance))
			{
				//Instantiates a new MotionControlHelper() object for the new turn segment
				m_TurnControl = new MotionControlHelper(m_targetAngle, ramp, maxSpeed, start, m_TurnSource, new DriveTurnPIDOutput(m_DriveTrain));
				m_TurnControl.setTargetDistance(m_targetAngle);
				
				//Instantiates a new MotionControlPIDController() object for the new turn segment using the previous MotionControlHelper()
				m_TurnPIDController = new MotionControlPIDController(TurnKp, TurnKi, TurnKd, m_TurnControl);
				m_TurnPIDController.setOutputRange(-1.0, 1.0);
				
				//Turns the MotionControlPID ON and it will continue to execute by itself until told otherwise.
				m_TurnPIDController.enable();	
				m_PIDEnabled = true;
				
				return true;
			}
			return false;
		}
		return true;
	}
	public boolean isStraightMotionFinished()
	{
		/*
		 * Called while waiting for the MotionControlPID to finish. The PID will be disabled when the end condition is met, and
		 * the return value indicates you can proceed to the next step.
		 * */
		if (Math.abs(m_DriveTrain.GetLeftDistance()-m_targetDistance) > Math.abs(m_straightTolerance))
		{
			m_StraightPIDController.disable();
			m_DriveTrain.ArcadeDrive(0, 0);
			m_PIDEnabled = false;
			return true;
		}
		return false;
	}
	public boolean isTurnMotionFinished()
	{
		/*
		 * Called while waiting for the MotionControlPID to finish. The PID will be disabled when the end condition is met, and
		 * the return value indicates you can proceed to the next step.
		 * */
		if (Math.abs(m_DriveTrain.GetAngle()-m_targetAngle) < m_turnTolerance)
		{
			m_TurnPIDController.disable();
			m_DriveTrain.ArcadeDrive(0, 0);
			m_PIDEnabled = false;
			return true;
		}
		return false;
	}
	public boolean isPIDEnabled()
	{
		return m_PIDEnabled;
	}
	public void DisablePIDControls()
	{
		if(m_TurnPIDController != null)
		{
			m_TurnPIDController.disable();
			m_StraightPIDOutput.disableRotationController();
			m_TurnPIDController.free();
		}
		if(m_StraightPIDController != null)
		{
			m_StraightPIDController.disable();
			m_StraightPIDController.free();
		}
	}
}
