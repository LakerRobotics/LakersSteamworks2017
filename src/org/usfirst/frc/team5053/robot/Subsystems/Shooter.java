package org.usfirst.frc.team5053.robot.Subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Talon;

public class Shooter implements Subsystem{

	private Talon m_Shooter;
	private Encoder m_Encoder;
	private PIDController m_PID;
	
	private double KP = 0.001;
	private double KI = 5.0E-4;
	private double KD = 0.0;
	
	public Shooter(Talon shooterTalon, Encoder shooterEncoder) {
		m_Shooter = shooterTalon;
		m_Encoder = shooterEncoder;
		m_Encoder.setPIDSourceType(PIDSourceType.kRate);
		
		m_PID = new PIDController(KP, KI, KD, m_Encoder, m_Shooter);
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
		m_Shooter.set(speed);
	}
	
	public void WriteDashboardData() {
		
	}
}