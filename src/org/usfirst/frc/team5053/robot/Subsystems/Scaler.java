package org.usfirst.frc.team5053.robot.Subsystems;

import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Scaler implements Subsystem {
	
	private Talon m_Scaler;
	
	public Scaler(Talon intakeTalon) {
		m_Scaler = intakeTalon;
	}
	
	public void SetTalonOutput(double speed) {
		m_Scaler.set(speed);
	}
	public void WriteDashboardData() {
		SmartDashboard.putNumber("intakeMotorPower", m_Scaler.getSpeed());
	}
}
