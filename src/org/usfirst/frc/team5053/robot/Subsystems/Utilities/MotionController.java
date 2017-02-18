package org.usfirst.frc.team5053.robot.Subsystems.Utilities;

import org.usfirst.frc.team5053.robot.Subsystems.DriveTrainMotionControl;
import org.usfirst.frc.team5053.robot.Subsystems.Utilities.MotionControlHelper;
import org.usfirst.frc.team5053.robot.Subsystems.Utilities.MotionControlPIDController;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class MotionController {
	DriveTrainMotionControl m_DriveTrain;
	MotionControlHelper m_StraightControl;
	MotionControlHelper m_SpinControl;
	MotionControlHelper m_ArchControl;
	
	MotionControlPIDController m_StraightPIDController;
	MotionControlPIDController m_SpinPIDController;
	MotionControlPIDController m_ArchPIDController;
	PIDController m_JoystickGyroPIDController;
	
	private double m_targetDistance;
	private double m_targetAngle;
	
	private double m_straightTolerance;
	private double m_turnTolerance;
//	private boolean m_PIDEnabled; --> replaced with m_PIDControlType
	private enum PIDControlType{DISTANCE, SPIN, ARCH, JOYSTICK_GYRO_ASSIST, OFF};
	private PIDControlType m_PIDControlType;
	
	private final double spinKp = 0.9;
	private final double spinKi = 0.0;
	private final double spinKd = 0.0;
	private final double archKp = 0.9;
	private final double archKi = 0.0;
	private final double archKd = 0.0;
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
		m_SpinPIDController = null;
		
		m_targetDistance = 0;
		m_targetAngle = 0;
		m_straightTolerance = 5;
		m_turnTolerance = 2;
		m_PIDControlType = PIDControlType.OFF;
		
	}
	/*
	 * distance specified in inches
	 * maxspeed specified in ft/sec
	 * ramp  specified in inches, provided how many inches to go from 0 to the maxspeed and also at end maxspeed to 0
	 */
	public boolean ExecuteStraightMotion(double distance, double maxspeed, double ramp)
	{
		if (m_PIDControlType==PIDControlType.OFF)
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
				m_StraightControl = new MotionControlHelper(convertedDistance, convertedRamp, convertedSpeed, start, m_StraightSource, new StraightMotionPIDOutput(m_DriveTrain, m_TurnSource, m_targetAngle));
				
				//Instantiates a new MotionControlPIDController() object for the new drive segment using the previous MotionControlHelper()
				m_StraightPIDController = new MotionControlPIDController(StraightKp, StraightKi, StraightKd, m_StraightControl);
				m_StraightPIDController.setAbsoluteTolerance(m_straightTolerance);
				m_StraightPIDController.setOutputRange(-1, 1);
				
				//Turns the MotionControlPID ON and it will continue to execute by itself until told otherwise.
				m_StraightPIDController.enable();
				m_PIDControlType = PIDControlType.DISTANCE;
				return true;
			}
			return false;
		}
		return true;
	}
	
	public boolean ExecuteTurnMotion(double turnAngle)
	{
		if (m_PIDControlType==PIDControlType.OFF)
		{
			//TODO remove this, it should still work since we are getting m_targetArgle to be TurnAngle + start
			//TODO remove this, it should still work since we are getting m_targetArgle to be TurnAngle + start
			//TODO remove this, it should still work since we are getting m_targetArgle to be TurnAngle + start
//			m_DriveTrain.ResetGyro();
			
			//Magic numbers need fixing
			double maxRPM = 60/*30*/;
			double ramp = 30/* 3.5 * maxRPM*/;
			
			double maxSpeed = maxRPM * 6; //360 Degrees/60 seconds to convert RPM to speed or degrees per second
			double start = m_DriveTrain.GetAngle();
			m_targetAngle = turnAngle + start;
			
			if (!(Math.abs(m_DriveTrain.GetAngle()-m_targetAngle) < m_turnTolerance))
			{
				//Instantiates a new MotionControlHelper() object for the new turn segment
				m_SpinControl = new MotionControlHelper(m_targetAngle, ramp, maxSpeed, start, m_TurnSource, new DriveTurnPIDOutput(m_DriveTrain));
				m_SpinControl.setTargetDistance(m_targetAngle);
				
				//Instantiates a new MotionControlPIDController() object for the new turn segment using the previous MotionControlHelper()
				m_SpinPIDController = new MotionControlPIDController(spinKp, spinKi, spinKd, m_SpinControl);
				m_SpinPIDController.setOutputRange(-1.0, 1.0);
				
				//Turns the MotionControlPID ON and it will continue to execute by itself until told otherwise.
				m_SpinPIDController.enable();	
				m_PIDControlType=PIDControlType.SPIN;
				
				return true;
			}
			return false;
		}
		return true;
	}
	public boolean ExecuteTurnMotion(double turnAngle, Joystick driverJoystick, int forwardPowerAxis)
	{
		if (m_PIDControlType==PIDControlType.OFF)
		{
			//TODO remove this, it should still work since we are getting m_targetArgle to be TurnAngle + start
			//TODO remove this, it should still work since we are getting m_targetArgle to be TurnAngle + start
			//TODO remove this, it should still work since we are getting m_targetArgle to be TurnAngle + start
//			m_DriveTrain.ResetGyro();
			
			//Magic numbers need fixing
			double maxRPM = 60/*30*/;
			double ramp = 30/* 3.5 * maxRPM*/;
			
			double maxSpeed = maxRPM * 6; //360 Degrees/60 seconds to convert RPM to speed or degrees per second
			double start = m_DriveTrain.GetAngle();
			m_targetAngle = turnAngle + start;
			
			if (!(Math.abs(m_DriveTrain.GetAngle()-m_targetAngle) < m_turnTolerance))
			{
				//Instantiates a new MotionControlHelper() object for the new turn segment
				m_SpinControl = new MotionControlHelper(m_targetAngle, ramp, maxSpeed, start, m_TurnSource, new DriveTurnPIDOutput(m_DriveTrain));
				m_SpinControl.setTargetDistance(m_targetAngle);
				
				//Instantiates a new MotionControlPIDController() object for the new turn segment using the previous MotionControlHelper()
				m_SpinPIDController = new MotionControlPIDController(spinKp, spinKi, spinKd, m_SpinControl);
				m_SpinPIDController.setOutputRange(-1.0, 1.0);
				
				//Turns the MotionControlPID ON and it will continue to execute by itself until told otherwise.
				m_SpinPIDController.enable();	
				m_PIDControlType=PIDControlType.SPIN;
				
				return true;
			}
			return false;
		}
		return true;
	}
	
	public boolean ExecuteArchMotion(double distance, double maxSpeed, double ramp, double radiusOfArch)
	{
		
		m_targetDistance = distance;
		m_DriveTrain.ResetEncoders();
		
		double start = 0;

		DriveArchPIDOutput motionControlArch ;
		if (m_PIDControlType==PIDControlType.OFF)
		{
//			m_DriveTrain.ResetGyro();
			
			double convertedDistance = distance;
			double convertedSpeed = maxSpeed * 12; // Inches
			double convertedRamp = ramp;
			
			motionControlArch = new DriveArchPIDOutput(m_DriveTrain, m_TurnSource, radiusOfArch);

			
			if (!(Math.abs(m_DriveTrain.GetAverageDistance()-m_targetDistance) < m_straightTolerance))
			{
				//Instantiates a new MotionControlHelper() object for the new Arch segment
				// motionControlForwardSpeed
				m_ArchControl = new MotionControlHelper(convertedDistance, convertedRamp, convertedSpeed, start, m_StraightSource, motionControlArch);
				
				//Instantiates a new MotionControlPIDController() object for the new turn segment using the previous MotionControlHelper()
				m_ArchPIDController = new MotionControlPIDController(archKp, archKi, archKd, m_ArchControl);
				m_ArchPIDController.setOutputRange(-1.0, 1.0);
				
				//Turns the MotionControlPID ON and it will continue to execute by itself until told otherwise.
				m_ArchPIDController.enable();	
				m_PIDControlType=PIDControlType.ARCH;
				return true;
			}
			return false;
		}
		return false;
	}
	public boolean ExecuteFowardPower_and_RotationSpeedMotion(Joystick joystick, int forwardPowerAxis, int rotationSpeedAxis )
	{
		double turnAngle=Double.MAX_VALUE; //Turn forever at the targetSpeed until someone says to stop turning at the target speed
		double maxRPM = 100;
		DriveArchPIDOutput motionControlRotationSpeed ;
		if (m_PIDControlType!=PIDControlType.JOYSTICK_GYRO_ASSIST)
		{
			DriveJoystickGyroPIDOutput joystickGyroPIDOutput= new DriveJoystickGyroPIDOutput(m_DriveTrain, m_TurnSource, joystick, forwardPowerAxis,rotationSpeedAxis, maxRPM);
			m_JoystickGyroPIDController = joystickGyroPIDOutput.createJoyStickGyroPIDController(m_TurnSource, joystickGyroPIDOutput);
				
			m_JoystickGyroPIDController.setOutputRange(-1.0, 1.0);
				
				//Turns the MotionControlPID ON and it will continue to execute by itself until told otherwise.
			m_JoystickGyroPIDController.enable();
			
			m_PIDControlType=PIDControlType.JOYSTICK_GYRO_ASSIST;
				
		}
		// always says it is not on target so it can constantly be trying to get the angle rotation correct.
		return false;
	}
//	/*
//	 * returns true if was able to adjust roationSpeed, for example if rotation speed PID not on then returns false
//	 */
//	public boolean setRotationSpeed(double rotationSpeedRPM){
//		double rotationSpeed = rotationSpeedRPM *360/60;// converts RPM to Degree/sec
//		if(this.m_PIDEnabled=true){
//			m_TurnControl.getRegularPIDControl().setSetpoint(rotationSpeed);
//			return false;
//		}
//		else{
//			return false;
//		}
//	}
	
	public boolean isStraightMotionFinished()
	{
		/*
		 * Called while waiting for the MotionControlPID to finish. The PID will be disabled when the end condition is met, and
		 * the return value indicates you can proceed to the next step.
		 * */
		//TODO Tolerance
		if (Math.abs(m_DriveTrain.GetLeftDistance()) > Math.abs(m_targetDistance))
		{
			m_StraightPIDController.disable();
			m_DriveTrain.ArcadeDrive(0, 0);
			m_PIDControlType=PIDControlType.OFF;
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
			m_SpinPIDController.disable();
			m_DriveTrain.ArcadeDrive(0, 0);
			m_PIDControlType=PIDControlType.OFF;
			return true;
		}
		return false;
	}
	public boolean isPIDEnabled()
	{
		return !(m_PIDControlType==PIDControlType.OFF);
	}
	public void DisablePIDControlsAll()
	{
		DisablePIDControlsTurn();
		DisablePIDControlsStraight();
		DisablePIDControlsArch();
		DisablePIDControlsJoystickGyroAssist();
	}
	public void DisablePIDControlsTurn()
	{
		if(m_SpinPIDController != null)
		{
			m_SpinPIDController.disable();
		}
	}
	public void DisablePIDControlsStraight()
	{
		if(m_StraightPIDController != null)
		{
			m_StraightPIDController.disable();
		}
	}
	public void DisablePIDControlsArch()
	{
		if(m_ArchPIDController != null)
		{
			m_ArchPIDController.disable();
		}
	}
	public void DisablePIDControlsJoystickGyroAssist()
	{
		if(this.m_JoystickGyroPIDController != null)
		{
			m_JoystickGyroPIDController.disable();
		}
	}
}
