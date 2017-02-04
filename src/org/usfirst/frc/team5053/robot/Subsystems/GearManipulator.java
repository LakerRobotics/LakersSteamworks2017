package org.usfirst.frc.team5053.robot.Subsystems;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class GearManipulator implements Subsystem {
	
	private Talon m_GearManipulator;
	private Encoder m_Encoder;
	private PIDController m_PID;

	private double KP = 15.0;
	private double KI = 0.05;
	private double KD = 0.0;
	
	private double HIGHEST_LIMIT = 0.0;
	private double LOWEST_LIMIT = 0.0;
	
	public GearManipulator(Talon armTalon, Encoder armEncoder) {
		m_GearManipulator = armTalon;
		m_Encoder = armEncoder;
		m_Encoder.setPIDSourceType(PIDSourceType.kRate);
		
		m_PID = new PIDController(KP, KI, KD, m_Encoder, m_GearManipulator);
	}
	
	public void EnablePID() {
		if(!m_PID.isEnabled())
		{
			m_PID.enable();
		}
	}
	
	public void DisablePID() {
		if(m_PID.isEnabled())
		{
			m_PID.disable();
		}
	}
	
	public boolean isPIDEnabled() {
		return m_PID.isEnabled();
	}
	
	public double GetPosition() {
		return m_Encoder.get();
	}
	
	public void SetTargetPosition(double target) {
		if(target < HIGHEST_LIMIT && target > LOWEST_LIMIT) 
		{
			m_PID.setSetpoint(target);
		}
	}
	
	public void SetTalonOutput(double speed) { 
		m_GearManipulator.set(speed);
	}
	
	public void WriteDashboardData() {
		SmartDashboard.putNumber("ArmPot", GetPosition());
	}
}
