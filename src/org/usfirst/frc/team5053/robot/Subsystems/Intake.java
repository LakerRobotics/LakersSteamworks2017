package org.usfirst.frc.team5053.robot.Subsystems;

import edu.wpi.first.wpilibj.SpeedController;

public class Intake implements Subsystem{
	
	private SpeedController m_Intake;
	
	public Intake(SpeedController intakeTalon) {
		m_Intake = intakeTalon;
	}
	
	public void SetTalonOutput(double speed) {
		m_Intake.set(speed);
	}
	public void WriteDashboardData() {
		// TODO Auto-generated method stub
		
	}
}
