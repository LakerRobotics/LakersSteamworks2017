package org.usfirst.frc.team5053.robot.Subsystems.Utilities;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.PowerDistributionPanel;

public class TalonCurrentLimited implements SpeedController {
	
	double m_CurrentLimitForward = 0;
	double m_CurrentLimitReverse = 0;
	int m_PowerDistributionBordSlot = 0;
	SpeedController m_SpeedController = null;
	public PowerDistributionPanel m_PDP = new PowerDistributionPanel();
	
	public TalonCurrentLimited(SpeedController speedController, int powerDistributionBordSlot,
			double currentLimitForward, double currentLimitReverse ) 
	{
		m_SpeedController = speedController;
		m_CurrentLimitForward = currentLimitForward;
		m_CurrentLimitReverse = currentLimitReverse;
		m_PowerDistributionBordSlot = powerDistributionBordSlot;

	}

	@Override
	public void pidWrite(double arg0) {
		m_SpeedController.pidWrite(arg0);
	}

	@Override
	public void disable() {
		m_SpeedController.disable();
	}

	@Override
	public double get() {
		return m_SpeedController.get();
	}

	@Override
	public boolean getInverted() {
		return m_SpeedController.getInverted();
	}

	@Override
	public void setInverted(boolean arg0) {
		m_SpeedController.setInverted(arg0);
	}

	@Override
	public void stopMotor() {
		m_SpeedController.stopMotor();
	}

	/**
	 * This is where we actually do something
	 */
	@Override
	public void set(double powerIn) {
		
		double current = m_PDP.getCurrent(m_PowerDistributionBordSlot);
		
		double powerOut = powerIn;
		if (current> m_CurrentLimitForward){
			powerOut = 0;
			
		}
		else if (current < m_CurrentLimitReverse){
			powerOut = 0;
		}
		
		m_SpeedController.set(powerOut);
	}
}
