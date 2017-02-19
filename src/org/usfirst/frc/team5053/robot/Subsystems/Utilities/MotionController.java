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
//TODO	MotionControlPIDController m_StraightJoystickPIDController;
	MotionControlPIDController m_SpinPIDController;
	MotionControlPIDController m_ArchPIDController;
	PIDController m_JoystickGyroPIDController;
	
	private double m_targetDistance;
	private double m_targetAngle;
	
	private double m_straightTolerance;
	private double m_turnTolerance;
	
//	private boolean m_PIDEnabled; --> replaced with m_PIDControlType
	private enum PIDControlType{DISTANCE, SPIN, ALIGN, ARCH, JOYSTICK_GYRO_ASSIST, OFF};
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
		m_straightTolerance = 2;
		m_turnTolerance = 2;
		m_PIDControlType = PIDControlType.OFF;
		
		//setup the PID Controllers once, think it will run better because it retains error history
		
	}
	/* Drives the robot a specified amount and then stops
	 * 
	 * distance specified in inches
	 * maxspeed specified in ft/sec
	 * ramp  specified in inches, provided how many inches to go from 0 to the maxspeed and also at end maxspeed to 0
	 */
	public boolean ExecuteStraightMotion(double distance, double maxspeed, double ramp)
	{
		System.out.print("MotionControl.ExecuteStraightMotion PIDControlType="+m_PIDControlType);
		if (m_PIDControlType!=PIDControlType.DISTANCE)
		{
			m_targetAngle = m_DriveTrain.GetAngle();
			m_targetDistance = distance;
			m_DriveTrain.ResetEncoders();
			
			double start = 0;
			
			double convertedDistance = distance;
			double convertedSpeed = maxspeed * 12; // Inches
			double convertedRamp = ramp;
			
			System.out.print("m_DriveTrain.GetLeftDistance()="+m_DriveTrain.GetLeftDistance());
			
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
		else // So PID Control running, see if we are in tolerance to target
		{
			if ((Math.abs(m_DriveTrain.GetLeftDistance()-m_targetDistance)<Math.abs(m_straightTolerance)))
			{
				return true;
			}
			else
			{
				return false;
			}
		}
//		return true;
	}
	public boolean isStraightMotionFinished()
	{
		/*
		 * Called while waiting for the MotionControlPID to finish. The PID will be disabled when the end condition is met, and
		 * the return value indicates you can proceed to the next step.
		 * */
		System.out.print("Math.abs(m_DriveTrain.GetAverageDistance()-m_targetDistance)="
				+Math.abs(m_DriveTrain.GetAverageDistance()-m_targetDistance)
				+"Math.abs(m_straightTolerance)=" + Math.abs(m_straightTolerance));
		if (Math.abs(m_DriveTrain.GetAverageDistance()-m_targetDistance) < Math.abs(m_straightTolerance))
		{
			System.out.println("DISABLEING PID FOR STRAIGHT");
			DisablePIDControlsStraight();
//			m_StraightPIDController.disable();
			m_DriveTrain.ArcadeDrive(0, 0);
			m_PIDControlType=PIDControlType.OFF;
			this.m_DriveTrain.ResetEncoders();
			return true;
		}
		return false;
	}

	
	/**
	 * Spins the robot the requested turn amount, and then stops
	 * @param turnAngle
	 * @return true if has spinned to the approriate angle
	 */
	public boolean ExecuteTurnMotion(double turnAngle)
	{
		if (m_PIDControlType!=PIDControlType.SPIN)
		{
			m_PIDControlType=this.KillAnyPIDControllerThatExist(PIDControlType.SPIN);
			this.DisablePIDControlsAll();
			
			double absoluteTolerance = 0.5; // Degrees
			
			double maxRPM = 40/*30*/;
			double ramp = 50/* 3.5 * maxRPM*/;
			
			double maxSpeed = maxRPM * 6; //360 Degrees/60 seconds to convert RPM to speed or degrees per second
			double start = m_DriveTrain.GetAngle();
			m_targetAngle = turnAngle + start;
			
				//Instantiates a new MotionControlHelper() object for the new turn segment
				m_SpinControl = new MotionControlHelper(m_targetAngle, ramp, maxSpeed, start, m_TurnSource, new DriveTurnPIDOutput(m_DriveTrain));
				m_SpinControl.setTargetDistance(m_targetAngle);
				
				//Instantiates a new MotionControlPIDController() object for the new turn segment using the MotionControlHelper()
				m_SpinPIDController = new MotionControlPIDController(spinKp, spinKi, spinKd, m_SpinControl);
				m_SpinPIDController.setOutputRange(-1.0, 1.0);
				
				//Turns the MotionControlPID ON and it will continue to execute by itself until told otherwise.
				m_SpinPIDController.enable();
		}
		return isTurnMotionFinished();
	}
	public boolean isTurnMotionFinished()
	{
		/*
		 * Called while waiting for the MotionControlPID to finish. The PID will be disabled when the end condition is met, and
		 * the return value indicates you can proceed to the next step.
		 * */
		System.out.print("Math.abs(m_DriveTrain.GetAngle()-m_targetAngle)="
				+Math.abs(m_DriveTrain.GetAngle()-m_targetAngle)
				+"Math.abs(m_turnTolerance)=" + Math.abs(m_turnTolerance));
		if (Math.abs(m_DriveTrain.GetAngle()-m_targetAngle) < Math.abs(m_turnTolerance))
		{
//			m_SpinPIDController.disable();
			this.DisablePIDControlsTurn();
			m_DriveTrain.ArcadeDrive(0, 0);
			m_PIDControlType=PIDControlType.OFF;
			return true;
		}
		return false;
	}

	/**
	 * 
	 * @param distance  to travel in inches
	 * @param maxSpeed  in ft/sec
	 * @param ramp      in inches
	 * @param radiusOfArch  The arc travel path of the robot
	 * @return true if it has completed the arc path
	 */
	public boolean ExecuteArchMotion(double distance, double maxSpeed, double ramp, double radiusOfArch)
	{
		m_targetDistance = distance;
		m_DriveTrain.ResetEncoders();
		
		double start = 0;

		DriveArchPIDOutput motionControlArch ;
		if (m_PIDControlType!=PIDControlType.ARCH)
		{
			m_PIDControlType=this.KillAnyPIDControllerThatExist(PIDControlType.ARCH);
			
			double convertedDistance = distance;
			double convertedSpeed = maxSpeed * 12; // convert to Inches/sec
			double convertedRamp = ramp; // in inches
			
			motionControlArch = new DriveArchPIDOutput(m_DriveTrain, m_TurnSource, radiusOfArch);

				//Instantiates a new MotionControlHelper() object for the new Arch segment
				// motionControlForwardSpeed
				m_ArchControl = new MotionControlHelper(convertedDistance, convertedRamp, convertedSpeed, start, m_StraightSource, motionControlArch);
				
				//Instantiates a new MotionControlPIDController() object for the new turn segment using the previous MotionControlHelper()
				m_ArchPIDController = new MotionControlPIDController(archKp, archKi, archKd, m_ArchControl);
				m_ArchPIDController.setOutputRange(-1.0, 1.0);
				
				//Turns the MotionControlPID ON and it will continue to execute by itself until told otherwise.
				m_ArchPIDController.enable();	
		}
		return m_ArchPIDController.onTarget();
	}
	
	
	/**
	 * 
	 * @param turnAngle
	 * @param driverJoystick
	 * @param forwardPowerAxis
	 * @return true if it is aligned
	 */
	public boolean ExecuteAlignMotion(double turnAngle, Joystick driverJoystick, int forwardPowerAxis)
	{
		if (m_PIDControlType!=PIDControlType.ALIGN)
		{
			m_PIDControlType=this.KillAnyPIDControllerThatExist(PIDControlType.ALIGN);
			
			double maxRPM = 40/*30*/;
			double ramp = 50/* 3.5 * maxRPM*/;
			
			double maxSpeed = maxRPM * 6; //360 Degrees/60 seconds to convert RPM to speed or degrees per second
			double start = m_DriveTrain.GetAngle();
			m_targetAngle = turnAngle + start;
			
				//Instantiates a new MotionControlHelper() object for the new turn segment
				m_SpinControl = new MotionControlHelper(m_targetAngle, ramp, maxSpeed, start, m_TurnSource, new DriveTurnPIDOutput(m_DriveTrain));
				m_SpinControl.setTargetDistance(m_targetAngle);
				
				//Instantiates a new MotionControlPIDController() object for the new turn segment using the previous MotionControlHelper()
				m_SpinPIDController = new MotionControlPIDController(spinKp, spinKi, spinKd, m_SpinControl);
				m_SpinPIDController.setOutputRange(-1.0, 1.0);
				
				//Turns the MotionControlPID ON and it will continue to execute by itself until told otherwise.
				m_SpinPIDController.enable();	
		}
		return m_SpinPIDController.onTarget();
	}

	public boolean doFowardPower_and_RotationSpeedMotion(Joystick joystick, int forwardPowerAxis, int rotationSpeedAxis )
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
	
	public boolean isPIDEnabled()
	{
		return !(m_PIDControlType==PIDControlType.OFF);
	}
	/**
	 * 
	 * @param turnOnThisPID
	 * @return returns the PID Controller that is now authorized to run.
	 */
	public PIDControlType KillAnyPIDControllerThatExist (PIDControlType turnOnThisPID)
	{
		// Note hope to in the future not null out existing controllers just disable them.
		
		DisablePIDControlsAll();
		
		// destroy all PIDs
		if(m_SpinPIDController         !=null){        m_SpinPIDController.free();m_SpinPIDController=null;};
		if(m_StraightPIDController     !=null){    m_StraightPIDController.free();m_StraightPIDController=null;};
		if(m_ArchPIDController         !=null){        m_ArchPIDController.free();m_ArchPIDController=null;};
		if(m_JoystickGyroPIDController !=null){m_JoystickGyroPIDController.free();m_JoystickGyroPIDController=null;};
		
		// was wanting to use this 
		if     (PIDControlType.SPIN                == turnOnThisPID){/*m_SpinPIDController.enable();*/}
		else if(PIDControlType.DISTANCE            == turnOnThisPID){/*m_StraightPIDController.enable();*/}
		else if(PIDControlType.ARCH                == turnOnThisPID){/*m_ArchPIDController.enable();*/}
		else if(PIDControlType.JOYSTICK_GYRO_ASSIST== turnOnThisPID){/*m_JoystickGyroPIDController.enable();*/}
		else if(PIDControlType.OFF                == turnOnThisPID);//dont enable anything;
		return turnOnThisPID;
	}
	public void DisablePIDControlsAll()
	{
		DisablePIDControlsTurn();
		DisablePIDControlsStraight();
		DisablePIDControlsArch();
		DisablePIDControlsJoystickGyroAssist();
		m_PIDControlType=PIDControlType.OFF;
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
