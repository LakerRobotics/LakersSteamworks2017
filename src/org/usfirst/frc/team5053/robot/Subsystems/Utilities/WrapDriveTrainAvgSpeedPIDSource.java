package org.usfirst.frc.team5053.robot.Subsystems.Utilities;

import org.usfirst.frc.team5053.robot.Subsystems.DriveTrain;
import org.usfirst.frc.team5053.robot.Subsystems.DriveTrainMotionControl;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

public class WrapDriveTrainAvgSpeedPIDSource implements PIDSource {
	DriveTrainMotionControl m_driveTrain ;
	PIDSourceType m_pidSourceType = PIDSourceType.kDisplacement;//Default as displacement
	
	public WrapDriveTrainAvgSpeedPIDSource(DriveTrainMotionControl driveTrain){
		m_driveTrain = driveTrain;
	}

	@Override
	public PIDSourceType getPIDSourceType() {
		return m_pidSourceType;
	}

	@Override
	public double pidGet() {
		if(m_pidSourceType == PIDSourceType.kRate){
			return m_driveTrain.GetAverageSpeed();
		}else{
			return m_driveTrain.GetAverageDistance();	
		}
	}

	@Override
	public void setPIDSourceType(PIDSourceType displacementOrRate) {
		 m_pidSourceType = displacementOrRate ;
	}

}
