package org.usfirst.frc.team5053.robot.Subsystems;

import java.util.HashMap;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Talon;

public class Arm implements Subsystem {
	
	private Talon m_Arm;
	private AnalogPotentiometer m_StringPot;
	private PIDController m_PID;
	
	public Arm(Talon armTalon, AnalogPotentiometer armStringPot) {
		m_Arm = armTalon;
		m_StringPot = armStringPot;
		m_StringPot.setPIDSourceType(PIDSourceType.kDisplacement);
		m_PID = new PIDController(15.0, 0.05, 0.0, m_StringPot, m_Arm);
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
		return m_StringPot.get();
	}
	public void SetTargetPosition(double target) {
		if(target < .353 && target > .179)
		{
			m_PID.setSetpoint(target);
		}
	}
	public void SetTalonOutput(double speed) { 
		m_Arm.set(speed);
	}
	public HashMap<String, Double> GetDashboardData() {
		HashMap<String, Double> data = new HashMap<String, Double>();
		data.put("ArmPot", GetPosition());
		return data;
	}
	
	
}
