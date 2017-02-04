package org.usfirst.frc.team5053.robot.Subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Talon;

public class Scaler implements Subsystem {
	
	private Talon m_Scaler;
	private Encoder m_Encoder;
	private PIDController m_PID;
	
	private double KP;
	private double KI;
	private double KD;
	
	public Scaler(Talon scalerTalon, Encoder scalerEncoder) {
		m_Scaler = scalerTalon;
		m_Encoder = scalerEncoder;
		

		m_Encoder.setPIDSourceType(PIDSourceType.kRate);
		m_PID = new PIDController(KP, KI, KD, m_Encoder, m_Scaler);
	}
	
	public void EnablePID() {
		m_PID.enable();
	}
	public void DisablePID() {
		m_PID.disable();
	}
	public boolean isPIDEnabled() {
		return m_PID.isEnabled();
	}
	public boolean ShooterOnTarget() {
		return m_PID.onTarget();
	}
	public void SetShooterSetpoint(double speed) {
		m_PID.setSetpoint(speed);
	}
	public void SetTalonOutput(double speed) {
		m_Scaler.set(speed);
	}
	
	@Override
	public void WriteDashboardData() {
		// TODO Auto-generated method stub
		
	}

}
