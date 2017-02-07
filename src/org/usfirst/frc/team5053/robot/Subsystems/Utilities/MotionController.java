package org.usfirst.frc.team5053.robot.Subsystems.Utilities;

import org.usfirst.frc.team5053.robot.Subsystems.DriveTrain;
import org.usfirst.frc.team5053.robot.Subsystems.DriveTrainMotionControl;
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

public class MotionController {
	DriveTrainMotionControl m_driveTrain;
	MotionControlHelper m_StraightControl;
	MotionControlHelper m_TurnControl;
	
	MotionControlPIDController m_StraightPIDController;
	MotionControlPIDController m_TurnPIDController;
	
	private double m_targetDistance;
	private double m_maxSpeed;
	private double m_distanceRamp;
	private double m_targetAngle;
	private double m_tolerance;
	
	private final double TurnKp = 0.0;
	private final double TurnKi = 0.0;
	private final double TurnKd = 0.0;
	private final double StraightKp = 0.0;
	private final double StraightKi = 0.0;
	private final double StraightKd = 0.0;
	
	PIDSource m_StraightSource;
	PIDSource m_TurnSource;
	
	public MotionController(DriveTrainMotionControl driveTrainMotionControl, PIDSource straightSource, PIDSource turnSource)
	{
		m_driveTrain = driveTrainMotionControl;
		m_StraightSource = straightSource;
		m_TurnSource = turnSource;
		
		m_StraightPIDController = null;
		m_TurnPIDController = null;
		
		
		m_targetDistance = 0;
		m_maxSpeed = 0;
		m_distanceRamp = 0;
		m_targetAngle = 0;
		m_tolerance = 5;
		
	}
	public void ExecuteStraightMotion(double distance, double maxspeed, double ramp)
	{
		m_targetAngle = m_driveTrain.GetAngle();
		m_driveTrain.ResetEncoders();
		
		double start = 0;
		
		m_StraightControl = new MotionControlHelper(distance, ramp, maxspeed, start, m_StraightSource, new StraightMotionPIDOutput(m_driveTrain, m_TurnSource, m_targetAngle));
		
		m_StraightPIDController = new MotionControlPIDController(StraightKp, StraightKi, StraightKd, m_StraightControl);
		m_StraightPIDController.setAbsoluteTolerance(m_tolerance);
		m_StraightPIDController.setOutputRange(-1, 1);
		
		m_StraightPIDController.enable();
	}
	public boolean isStraightMotionFinished()
	{
		if (Math.abs(m_driveTrain.GetRightDistance()-m_targetDistance) < m_tolerance)
		{
			m_StraightPIDController.disable();
			m_driveTrain.ArcadeDrive(0, 0);;
			return true;
		}
		return false;
	}
}
