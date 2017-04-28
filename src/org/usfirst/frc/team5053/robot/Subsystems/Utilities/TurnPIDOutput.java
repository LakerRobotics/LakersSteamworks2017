package org.usfirst.frc.team5053.robot.Subsystems.Utilities;

import org.usfirst.frc.team5053.robot.Subsystems.DriveTrain;

import edu.wpi.first.wpilibj.PIDOutput;

public class TurnPIDOutput implements PIDOutput {
	
	private DriveTrain m_DriveTrain;
	private double POWER = 0.25;
	
	public TurnPIDOutput(DriveTrain driveTrain) {
		m_DriveTrain = driveTrain;
	}
	
	public void pidWrite(double rotationPower) {
		//Left | Right
		if(rotationPower < 0)
			rotationPower = rotationPower - POWER;
		if(rotationPower > 0)
			rotationPower = rotationPower + POWER;
		else
			rotationPower = 0;
		
		m_DriveTrain.tankDrive(-rotationPower, rotationPower);
	}
}