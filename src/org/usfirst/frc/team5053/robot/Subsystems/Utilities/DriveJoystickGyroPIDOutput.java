/**
 * 
 */
package org.usfirst.frc.team5053.robot.Subsystems.Utilities;

import org.usfirst.frc.team5053.robot.Subsystems.DriveTrainMotionControl;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * @author richard.topolewski
 *
 */
public class DriveJoystickGyroPIDOutput implements PIDOutput  {

	DriveTrainMotionControl m_driveTrain ;
	
	PIDSource m_gyro; 
	
	double rotationPower = 0.0d;
	private double forwardPower  = 0.0d;
	double m_archRadius    = 0.0d;
	PIDController m_pidControlRotationGyro = null;

	
	Joystick m_joystick ;
	int m_RotationSpeedAxis;
	int m_ForwardPowerAxis; 
	double m_maxRotation;  // in Degrees/sec
	/**
	 * @param driveTrain
	 * @param theGyro
	 * @param archRadius
	 */
	public DriveJoystickGyroPIDOutput(DriveTrainMotionControl driveTrain, PIDSource theGyro, Joystick joystick, int forwardPowerAxis, int rotationSpeedAxis, double maxRotationRPM) {
		m_driveTrain = driveTrain;
		m_gyro = theGyro;

		m_joystick = joystick;
		m_RotationSpeedAxis = rotationSpeedAxis;
		m_ForwardPowerAxis = forwardPowerAxis;
		
		m_maxRotation = maxRotationRPM*360/6;

//		m_pidControlRotationGyro = createArchPIDController( m_gyro, this);
	}
	


	protected synchronized double getRotationPower() {
		return rotationPower;
	}


	@Override
	public synchronized void pidWrite(double rotationPower) {
		double a_forwardPower = this.m_joystick.getRawAxis(m_ForwardPowerAxis) ;			
				 
		//Adjust the rotation power to be based on the RotationSpeedAxis
		double joystickTurnAxisReading = m_joystick.getRawAxis(m_RotationSpeedAxis);
		m_pidControlRotationGyro.setSetpoint(m_maxRotation*joystickTurnAxisReading);
				
	    SmartDashboard.putNumber("DriveJoystickGyroPIDOutput Motor Output",forwardPower);
	    SmartDashboard.putNumber("DriveJoystickGyroPIDOutput RotationPower", rotationPower);
    	double leftPower; 
    	double rightPower;
    	
    	// Reduce forward power so can get full and even turning effect
    	// also may help if quickly reduce from full throttle, to avoid a jerk in the rotation as the PID would convert from 1/2 to all all rotation power
    	// .
    	double forwardPower;
    	if (Math.abs(a_forwardPower + rotationPower) > 1.0 || Math.abs(a_forwardPower - rotationPower) > 1.0){
    		if(a_forwardPower > 0){
    			forwardPower = 1 - Math.abs(rotationPower);
    		}
    		else{
    			forwardPower = -1 + Math.abs(rotationPower);   			
    		}
    	}else{
    		forwardPower = a_forwardPower;
    	}
    	
    	leftPower = forwardPower-rotationPower;
    	rightPower = forwardPower+rotationPower;
    	m_driveTrain.tankDrive(leftPower, rightPower);

	}
	

	public  PIDController createJoyStickGyroPIDController(PIDSource gyro, PIDOutput rotationPower) {
		
		
		final double Kp = 1/100; // so at denominator off in the spin-Rate the power will reach the max
	    final double Ki = 0.0000;
	    final double Kd = 0.0;
	 
	    MotionControlPIDController localRotationSpeedPID;

        //TODO have the targetAngle stop the adjustment from this.pidWrite(..)
	    gyro.setPIDSourceType(PIDSourceType.kRate);

        PIDController gyroRotationPID = new PIDController(Kp,Ki,Kd, gyro, rotationPower );
        gyroRotationPID.setOutputRange(-1.0, 1.0);
        gyroRotationPID.enable();
	    return gyroRotationPID;
	}
	
	
    public double getForwardPower() {
		return forwardPower;
	}


	
	

}
