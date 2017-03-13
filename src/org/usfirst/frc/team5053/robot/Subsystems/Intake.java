package org.usfirst.frc.team5053.robot.Subsystems;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intake implements Subsystem{
	
	private Talon m_Intake;
	private double m_Speed;
	
	public Intake(SpeedController intakeTalon) {
		m_Intake = (Talon) intakeTalon;
	}
	
	public void SetTalonOutput(double speed) {
		m_Intake.set(speed);
	}
	public void WriteDashboardData() {
		SmartDashboard.putNumber("Intake Speed", m_Speed);
	}
}
