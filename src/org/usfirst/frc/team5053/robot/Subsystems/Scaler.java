package org.usfirst.frc.team5053.robot.Subsystems;

import edu.wpi.first.wpilibj.Talon;

public class Scaler implements Subsystem {
	
	private Talon m_Scaler;
	
	public Scaler(Talon scalerTalon) {
		m_Scaler = scalerTalon;
	}
	public void SetTalonOutput(double speed) {
		m_Scaler.set(speed);
	}
	
	@Override
	public void WriteDashboardData() {
		// TODO Auto-generated method stub
	}
}
