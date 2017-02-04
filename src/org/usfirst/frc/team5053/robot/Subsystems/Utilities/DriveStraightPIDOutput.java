package org.usfirst.frc.team5053.robot.Subsystems.Utilities;

import org.usfirst.frc.team5053.robot.Subsystems.DriveTrainMotionControl;

import edu.wpi.first.wpilibj.PIDOutput;

public class DriveStraightPIDOutput implements PIDOutput {
	
	private DriveTrainMotionControl m_DriveTrain;
	
	public DriveStraightPIDOutput(DriveTrainMotionControl driveTrain) {
		m_DriveTrain = driveTrain;
	}
	
	public void pidWrite(double rotationPower) {
		//Left | Right
		m_DriveTrain.tankDrive(-rotationPower, rotationPower);
	}
}
