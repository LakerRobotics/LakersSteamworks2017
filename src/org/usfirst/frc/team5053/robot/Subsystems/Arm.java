package org.usfirst.frc.team5053.robot.Subsystems;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Arm implements Subsystem {
	
	private Talon m_Arm;
	private AnalogPotentiometer m_StringPot;
	private PIDController m_PID;

	private double KP = 15.0;
	private double KI = 0.05;
	private double KD = 0.0;
	
	private double HIGHEST_POSITION = .353;
	private double LOWEST_POSITION = .179;
	
	public Arm(Talon armTalon, AnalogPotentiometer armStringPot) {
		m_Arm = armTalon;
		m_StringPot = armStringPot;
		m_StringPot.setPIDSourceType(PIDSourceType.kDisplacement);
		
		m_PID = new PIDController(KP, KI, KD, m_StringPot, m_Arm);
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
		if(target < HIGHEST_POSITION && target > LOWEST_POSITION) 
		{
			m_PID.setSetpoint(target);
		}
	}
	public void SetTalonOutput(double speed) { 
		m_Arm.set(speed);
	}
	public void WriteDashboardData() {
		SmartDashboard.putNumber("ArmPot", GetPosition());
	}
	
	
}
