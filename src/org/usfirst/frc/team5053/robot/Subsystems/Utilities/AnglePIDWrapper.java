package org.usfirst.frc.team5053.robot.Subsystems.Utilities;
import org.usfirst.frc.team5053.robot.Subsystems.DriveTrain;

import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

public class AnglePIDWrapper implements PIDOutput, PIDSource {
	
	private DriveTrain m_drivetrain;
	private final double OUTPUT_RANGE = .5;
	
	public AnglePIDWrapper(DriveTrain drivetrain)
	{
		m_drivetrain = drivetrain;
	}

	@Override
	public PIDSourceType getPIDSourceType() {
		return PIDSourceType.kDisplacement;
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
		m_drivetrain.SetTurn(output*OUTPUT_RANGE);
	}

}
