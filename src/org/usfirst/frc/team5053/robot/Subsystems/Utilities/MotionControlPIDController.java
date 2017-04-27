package org.usfirst.frc.team5053.robot.Subsystems.Utilities;

import java.util.TimerTask;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GyroBase;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.BoundaryException;

public class MotionControlPIDController extends PIDController {
	MotionControlHelper m_motionControlHelper; 
	/**
	 * 
	 * @return
	 * @override
	 * @throws Exception
	 */
	public double getRate() throws Exception
	{
		// Set the PIDSource to return Rate and then get the Rate.
		
		m_motionControlHelper.getM_source().setPIDSourceType(PIDSourceType.kRate);
		SmartDashboard.putNumber("Motion Control Rate", this.getRate());
		return m_motionControlHelper.getM_source().pidGet();
	}
	
	public MotionControlHelper getMotionControlHelper()
	{
		return m_motionControlHelper;
	}


	public MotionControlPIDController(double Kp, double Ki, double Kd, MotionControlHelper motionControl) 
	{
		super(Kp, Ki, Kd, motionControl.getM_source(), motionControl.getM_output());
		m_motionControlHelper = motionControl;
		motionControl.setRegularPIDControl(this);// to let the motionControl adjust the Rate, ie do the motion control
	}
	
	public MotionControlPIDController(double Kp, double Ki, double Kd, MotionControlHelper motionControl, double period) 
	{
		super(Kp, Ki, Kd, motionControl.getM_source(), motionControl.getM_output(), period);
		m_motionControlHelper = motionControl;
		motionControl.setRegularPIDControl(this);// to let the motionControl adjust the Rate, ie do the motion control
	}

	public MotionControlPIDController(double Kp, double Ki, double Kd, double Kf, MotionControlHelper motionControl) 
	{
		super(Kp, Ki, Kd, Kf, motionControl.getM_source(), motionControl.getM_output());
		m_motionControlHelper = motionControl;
		motionControl.setRegularPIDControl(this);// to let the motionControl adjust the Rate, ie do the motion control
	}

	public MotionControlPIDController(double Kp, double Ki, double Kd, double Kf, double period, MotionControlHelper motionControl) 
	{
		super(Kp, Ki, Kd, Kf, motionControl.getM_source(), motionControl.getM_output(), period);
		m_motionControlHelper = motionControl; 
		motionControl.setRegularPIDControl(this);// to let the motionControl adjust the Rate, ie do the motion control
	}
}
