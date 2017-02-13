package org.usfirst.frc.team5053.robot.Subsystems;

import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Mixer implements Subsystem{
	
	private Talon m_Mixer;
	private double m_Speed;
	
	public Mixer(Talon intakeTalon) {
		m_Mixer = intakeTalon;
	}
	
	public void SetTalonOutput(double speed) {
		m_Mixer.set(speed);
	}
	public void WriteDashboardData() {
		SmartDashboard.putNumber("Mixer Speed", m_Speed);
	}
}
