package org.usfirst.frc.team5053.robot.Subsystems.Utilities;

import org.usfirst.frc.team5053.robot.Subsystems.DriveTrain;

import edu.wpi.first.wpilibj.PIDOutput;

public class DriveStraightPIDOutput implements PIDOutput {
	
	private DriveTrain m_DriveTrain;
	
	public DriveStraightPIDOutput(DriveTrain driveTrain) {
		m_DriveTrain = driveTrain;
	}
	
	public void pidWrite(double rotationPower) {
		//Left | Right
		m_DriveTrain.tankDrive(-rotationPower, rotationPower);
	}
}
