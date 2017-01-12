package org.usfirst.frc.team5053.robot.Subsystems;

import edu.wpi.first.wpilibj.Talon;

/**
 * Subsystem Template to base new subsystems off of.
 */
public class SubsystemTemplate implements Subsystem
{
	//Member variables! Will be made private *most* of the time.
	private Talon m_SubsystemMotor;
	
	//A Constructor
	/** I'm a Subsystem Constructor!
	 * 
	 */
	public SubsystemTemplate()
	{
		
	}
	
	//Overloaded Constructor
	
	/** I'm a Subsystem Constructor that takes a motor on initialization!
	 *  This is where you would assign motor to a member variable in the subsystem
	 */
	public SubsystemTemplate(Talon motor)
	{
		
		
		m_SubsystemMotor = motor;
	}
	
	//Generic Set Method
	/**
	 * This set method sets the speed of the motor to the value of the passed argument.
	 */
	void setMethod(double value)
	{
		
		m_SubsystemMotor.set(value);
	}
	
	//Generic Get Method
	/**
	 * This get method returns the speed of the motor.
	 */
	double getMethod()
	{
		
		return m_SubsystemMotor.getSpeed();
	}

	@Override
	public void WriteDashboardData() {
		// TODO Auto-generated method stub
	}
}
