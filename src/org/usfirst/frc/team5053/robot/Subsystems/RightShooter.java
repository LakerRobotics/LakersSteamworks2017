package org.usfirst.frc.team5053.robot.Subsystems;

import java.util.HashMap;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Talon;

public class RightShooter implements Subsystem{

	private Talon m_Shooter;
	private Encoder m_Encoder;
	private PIDController m_PID;
	
	public RightShooter(Talon shooterTalon, Encoder shooterEncoder) {
		m_Shooter = shooterTalon;
		m_Encoder = shooterEncoder;
		m_Encoder.setPIDSourceType(PIDSourceType.kRate);
		m_PID = new PIDController(0.001, 5.0E-4, 0.0, m_Encoder, m_Shooter);
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
	public HashMap<String, Double> GetDashboardData() {
		// TODO Auto-generated method stub
		return null;
	}
}