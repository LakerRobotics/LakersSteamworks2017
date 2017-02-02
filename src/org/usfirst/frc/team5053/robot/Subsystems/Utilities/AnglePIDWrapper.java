package org.usfirst.frc.team5053.robot.Subsystems.Utilities;
import org.usfirst.frc.team5053.robot.Subsystems.DriveTrain;

import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

public class AnglePIDWrapper implements PIDOutput, PIDSource {
	
	private DriveTrain m_drivetrain;
	public AnglePIDWrapper(DriveTrain drivetrain)
	{
		m_drivetrain = drivetrain;
	}

	@Override
	public PIDSourceType getPIDSourceType() {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public double pidGet() {
		return (m_drivetrain.GetAngle());
	}

	@Override
	public void setPIDSourceType(PIDSourceType arg0) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void pidWrite(double output) {
		m_drivetrain.SetTurn(output);
	}

}
