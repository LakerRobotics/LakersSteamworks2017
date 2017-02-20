package org.usfirst.frc.team5053.robot.Subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter implements Subsystem {

	private Talon m_Shooter;
	private Encoder m_Encoder;
	private PIDController m_PID;
	
	private double KP = 0.001;
	private double KI = 0.005;
	private double KD = 0.0;
	
	private double PERCENT_TOLERANCE = 0.5;
	
	public Shooter(Talon shooterTalon, Encoder shooterEncoder) {
		m_Shooter = shooterTalon;
		m_Encoder = shooterEncoder;
		
		m_PID = new PIDController(KP, KI, KD, m_Encoder, m_Shooter);
		m_PID.setOutputRange(0.0, 1.0);
		m_PID.setAbsoluteTolerance(PERCENT_TOLERANCE);
	}
	
	public void EnablePID() {
		if (!isPIDEnabled())
		{
			m_PID.enable();
		}
		
	}
	public void DisablePID() {
		m_PID.disable();
		if (isPIDEnabled())
		{
			m_PID.disable();
			m_PID.free();
		}
	}
	public boolean isPIDEnabled() {
		return m_PID.isEnabled();
	}
	public boolean ShooterOnTarget() {
		if(Math.abs(m_Encoder.pidGet()) >= Math.abs(m_PID.getSetpoint()*(1-PERCENT_TOLERANCE)) && Math.abs(m_Encoder.pidGet()) <= Math.abs(m_PID.getSetpoint()*(1+PERCENT_TOLERANCE)))
		{
			return true;
		}
		else
		{
			return false;
		}
	}
	public void SetShooterSetpoint(double speed) {
		m_PID.setSetpoint(speed);
	}
	public void SetTalonOutput(double speed) {
		m_Shooter.set(speed);
	}
	public void WriteDashboardData() {
		SmartDashboard.putNumber("Shooter Setpoint", m_PID.getSetpoint());
		SmartDashboard.putNumber("shooterDriveEncoder", m_Encoder.pidGet());
	}
}