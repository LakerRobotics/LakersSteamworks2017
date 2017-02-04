package org.usfirst.frc.team5053.robot.Subsystems.Utilities;

import org.usfirst.frc.team5053.robot.Subsystems.DriveTrainMotionControl;

import edu.wpi.first.wpilibj.PIDOutput;

public class TurnPIDOutput implements PIDOutput {
	
	private DriveTrainMotionControl m_DriveTrain;
	
	public TurnPIDOutput(DriveTrainMotionControl driveTrain) {
		m_DriveTrain = driveTrain;
	}
	
	public void pidWrite(double rotationPower) {
		//Left | Right
		m_DriveTrain.tankDrive(-rotationPower, rotationPower);
	}
}