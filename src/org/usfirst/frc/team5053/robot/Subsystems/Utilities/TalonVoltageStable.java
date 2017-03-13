package org.usfirst.frc.team5053.robot.Subsystems.Utilities;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.PowerDistributionPanel;

public class TalonVoltageStable implements SpeedController {
	
	private SpeedController m_SpeedController = null;
	private PowerDistributionPanel m_PDP = new PowerDistributionPanel();
	private final double AVERAGE_VOLTAGE = 12.7;
	
	public TalonVoltageStable(SpeedController speedController) {
		m_SpeedController = speedController;
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
	 * Adjust output for low battery voltage
	 */
	@Override
	public void set(double powerIn) {
		double currentBatteryVoltage = m_PDP.getVoltage();
		
		double adjustedOutput = (powerIn*AVERAGE_VOLTAGE)/currentBatteryVoltage;
		
		m_SpeedController.set(adjustedOutput);
	}

}
