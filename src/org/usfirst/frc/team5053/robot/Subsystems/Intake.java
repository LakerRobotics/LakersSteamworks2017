package org.usfirst.frc.team5053.robot.Subsystems;

import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intake implements Subsystem{
	
	private Talon m_Intake;
	
	public Intake(Talon intakeTalon) {
		m_Intake = intakeTalon;
	}
	
	public void SetTalonOutput(double speed) {
		m_Intake.set(speed);
	}
	
	public void WriteDashboardData() {
		SmartDashboard.putNumber("intakeMotorPower", m_Intake.getSpeed());
	}
}
