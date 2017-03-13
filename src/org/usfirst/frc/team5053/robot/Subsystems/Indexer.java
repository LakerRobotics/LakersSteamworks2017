package org.usfirst.frc.team5053.robot.Subsystems;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Indexer implements Subsystem{
	
	private Talon m_Indexer;
	private double m_Speed;
	
	public Indexer(SpeedController intakeTalon) {
		m_Indexer = (Talon) intakeTalon;
	}
	
	public void SetTalonOutput(double speed) {
		m_Indexer.set(speed);
	}
	public void WriteDashboardData() {
		SmartDashboard.putNumber("Indexer Speed", m_Speed);
	}
}
