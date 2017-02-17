package org.usfirst.frc.team5053.robot.Subsystems.Utilities;

import edu.wpi.first.wpilibj.PIDOutput;

 class WrapArchPIDOutput implements PIDOutput {

    private DriveArchPIDOutput m_rotationPowerDestination;

    // constructor
    public WrapArchPIDOutput(DriveArchPIDOutput rotationPowerDesintation) {
        if (rotationPowerDesintation == null) {
            throw new NullPointerException("Given rotationPowerDestination was null");
        }
        else{
            m_rotationPowerDestination = rotationPowerDesintation;            	
        }
    }

	@Override
	public void pidWrite(double rotationPower) {
		m_rotationPowerDestination.setRotationPower(rotationPower);
	}

}