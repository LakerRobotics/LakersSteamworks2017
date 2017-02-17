package org.usfirst.frc.team5053.robot.Subsystems.Utilities;

import org.usfirst.frc.team5053.robot.Subsystems.DriveTrainMotionControl;

import edu.wpi.first.wpilibj.PIDOutput;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveTurnPIDOutput implements PIDOutput {

	DriveTrainMotionControl m_DriveTrain;
	private final double POWER_MODIFIER = .5;// Why is there a power modifier, 
	                                         // seems should be eliminated to allow full power, can do same thing in PID control if really necciary but should 
	
	public DriveTurnPIDOutput(DriveTrainMotionControl driveTrain) {
	    SmartDashboard.putString("DriveSpinPIDOutput", "constructor called");
	    m_DriveTrain = driveTrain;
	}

	@Override
	public void pidWrite(double output) {
		m_DriveTrain.tankDrive(-output*POWER_MODIFIER,output*POWER_MODIFIER); 
		System.out.println("DriveSpinPIDOutput Rotation Motor Output:"+output);
		SmartDashboard.putNumber("DriveSpinPIDOutput Rotation Motor Output",output); 
	}

}