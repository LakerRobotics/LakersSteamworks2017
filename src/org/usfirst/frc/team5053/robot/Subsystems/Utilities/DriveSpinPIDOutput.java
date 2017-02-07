package org.usfirst.frc.team5053.robot.Subsystems.Utilities;

import org.usfirst.frc.team5053.robot.Robot;

import edu.wpi.first.wpilibj.PIDOutput;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveSpinPIDOutput implements PIDOutput {


	public DriveSpinPIDOutput() {
	    SmartDashboard.putString("DriveSpinPIDOutput", "constructor called");
	}

	@Override
	public void pidWrite(double output) {
		Robot.getDriveTrain().getRobotDrive().tankDrive(-output,output); 
		System.out.println("DriveSpinPIDOutput Rotation Motor Output:"+output);
		SmartDashboard.putNumber("DriveSpinPIDOutput Rotation Motor Output",output); 
	}

}
