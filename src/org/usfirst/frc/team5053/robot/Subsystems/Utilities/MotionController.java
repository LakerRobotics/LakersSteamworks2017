package org.usfirst.frc.team5053.robot.Subsystems.Utilities;

import org.usfirst.frc.team5053.robot.Subsystems.DriveTrainMotionControl;
import org.usfirst.frc.team5053.robot.Subsystems.Utilities.MotionControlHelper;
import org.usfirst.frc.team5053.robot.Subsystems.Utilities.MotionControlPIDController;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class MotionController {
	private DriveTrainMotionControl m_DriveTrain;
	
	private MotionControlHelper m_StraightControl;
	private MotionControlHelper m_TurnControl;
	private MotionControlHelper m_ArcControl;
	
	private MotionControlPIDController m_StraightPIDController;
	private StraightMotionPIDOutput m_StraightPIDOutput;
	private MotionControlPIDController m_TurnPIDController;
	private MotionControlPIDController m_ArcPIDController;
	
	private double m_targetDistance;
	private double m_targetAngle;
	private double m_straightTolerance;
	private double m_turnTolerance;
	private boolean m_PIDEnabled;
	
	private final double TurnKp = 0.005;
	private final double TurnKi = 0.0001;
	private final double TurnKd = 0.0;
	
	private final double StraightKp = 0.1;
	private final double StraightKi = 0.0001;
	private final double StraightKd = 0.0;

	private final double ArcKp = 0.1;
	private final double ArcKi = 0.0001;
	private final double ArcKd = 0.0;
	
	private PIDSource m_StraightSource;
	private PIDSource m_TurnSource;
	
	
	
	public MotionController(DriveTrainMotionControl driveTrainMotionControl, PIDSource straightSource, PIDSource turnSource)
	{
		m_DriveTrain = driveTrainMotionControl;
		m_StraightSource = straightSource;
		m_TurnSource = turnSource;
		
		m_StraightPIDController = null;
		m_StraightPIDOutput = null;
		m_TurnPIDController = null;
		m_ArcPIDController = null;
		
		m_targetDistance = 0;
		m_targetAngle = 0;
		m_straightTolerance = 3;
		m_turnTolerance = 1;
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
			
			double maxRPM = 30/*30*/;
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
	/**
	 * 
	 * @param distance  to travel in inches
	 * @param maxSpeed  in ft/sec
	 * @param ramp      in inches
	 * @param radiusOfArch  The arc travel path of the robot
	 * @return true if it has completed the arc path
	 */
	public boolean ExecuteArcMotion(double distance, double maxSpeed, double ramp, double radiusOfArc)
	{
		m_targetDistance = distance;
		m_DriveTrain.ResetEncoders();
		
		double start = 0;

		ArcMotionPIDOutput motionControlArc;
		
		if (!isPIDEnabled())
		{
			double convertedDistance = distance;
			double convertedSpeed = maxSpeed * 12; // convert to Inches/sec
			double convertedRamp = ramp; // in inches
			
			motionControlArc = new ArcMotionPIDOutput(m_DriveTrain, m_TurnSource, radiusOfArc);

			//Instantiates a new MotionControlHelper() object for the new Arch segment
			// motionControlForwardSpeed
			m_ArcControl = new MotionControlHelper(convertedDistance, convertedRamp, convertedSpeed, start, m_StraightSource, motionControlArc);
			
			//Instantiates a new MotionControlPIDController() object for the new turn segment using the previous MotionControlHelper()
			m_ArcPIDController = new MotionControlPIDController(ArcKp, ArcKi, ArcKd, m_ArcControl);
			m_ArcPIDController.setOutputRange(-1.0, 1.0);
			
			//Turns the MotionControlPID ON and it will continue to execute by itself until told otherwise.
			m_ArcPIDController.enable();
			return true;
		}
		return true;
	}
	public boolean isStraightMotionFinished()
	{
		/*
		 * Called while waiting for the MotionControlPID to finish. The PID will be disabled when the end condition is met, and
		 * the return value indicates you can proceed to the next step.
		 * */
		SmartDashboard.putNumber("Distance Left", m_DriveTrain.GetLeftDistance());
		SmartDashboard.putNumber("Target distance", m_targetDistance);
		SmartDashboard.putNumber("Straight Tolerance", m_straightTolerance);
		
		//TODO Verify this tolerance works... it should...
		if (Math.abs(m_DriveTrain.GetLeftDistance()) >= Math.abs(m_targetDistance - m_straightTolerance))
		{
			//Always tripped
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
	public boolean isArcMotionFinished()
	{
		/*
		 * Called while waiting for the MotionControlPID to finish. The PID will be disabled when the end condition is met, and
		 * the return value indicates you can proceed to the next step.
		 * */
		SmartDashboard.putNumber("Distance Left", m_DriveTrain.GetLeftDistance());
		SmartDashboard.putNumber("Target distance", m_targetDistance);
		SmartDashboard.putNumber("Straight Tolerance", m_straightTolerance);
		
		//TODO Verify this tolerance works... it should...
		if (Math.abs(m_DriveTrain.GetAverageDistance() - m_targetDistance) <= Math.abs(m_straightTolerance))
		{
			//Always tripped
			m_ArcPIDController.disable();
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
		}
		if(m_StraightPIDController != null)
		{
			m_StraightPIDController.disable();
			m_StraightPIDOutput.disableRotationController();
		}
	}
}
