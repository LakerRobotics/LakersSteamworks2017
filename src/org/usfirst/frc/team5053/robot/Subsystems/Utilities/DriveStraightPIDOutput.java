package org.usfirst.frc.team5053.robot.Subsystems.Utilities;

import org.usfirst.frc.team5053.robot.Robot;
import org.usfirst.frc.team5053.robot.Subsystems.DriveTrain;
//FromRudy import org.usfirst.frc5053.FRC2016Stronghold.MotionControlHelper;
//FromRudy import org.usfirst.frc5053.FRC2016Stronghold.MotionControlPIDController;
//FromRudy import org.usfirst.frc5053.FRC2016Stronghold.RobotMap;

import edu.wpi.first.wpilibj.PIDOutput;

	
	import edu.wpi.first.wpilibj.PIDOutput;
	import edu.wpi.first.wpilibj.PIDSource;
	import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;

	import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

	/**
	 * 
	 * @author Rich Topolewski
	 * 
	 * Used to take the speed calculated from the PID control and pass it to the drive train, 
	 * and also adjust the speed going to the wheels to drive straight
	 *
	 */
	public class DriveStraightPIDOutput implements PIDOutput {
		RobotDrive robotDrive = Robot.m_DriveTrain;
		Gyro gyro = Robot.m_RobotSensors.m_Gyro;
		
		// This is just a simple P control, Proportional control of the line follow
		// if we assume angle is in degrees and if we were off by 20 Degrees then we would want how much correction
		// for example id Kp is 0.025 at 20 degrees we would have 0.5 or half the power toward rotating the robot 
		double Kp = 0d/20d; //0.025;// 
		Gyro m_gyro; 
		double m_targetAngle = 0.0d;
		double rotationPower = 0.0d;

		public DriveStraightPIDOutput(Gyro theGyro, double targetAngle) {
//		    SmartDashboard.putString("RobotDriveSpinPIDOutput", "constructor called");
			m_gyro = theGyro;
			m_targetAngle = targetAngle;
			double slowRotation = m_targetAngle + 90;
			WrapRotationPIDOutput wrappedRotationPIDOutput =  new WrapRotationPIDOutput(this);
			
			MotionControlPIDController rotationPID = createRotationPIDController(m_targetAngle, slowRotation, wrappedRotationPIDOutput);
			
			//WrapRotationPIDInput  wrapRotationPIDInput = new WrapRotationPIDOutput(rotationPID, (PIDSource) m_gyro);
		}

		protected synchronized double getRotationPower() {
			return rotationPower;
		}


		protected synchronized void setRotationPower(double rotationPower) {
			this.rotationPower = rotationPower;
		}


		@Override
		public synchronized void pidWrite(double motorPower) {
		    //rotationPower
		   	//double rotationPower = 0;
		   	//RobotMap.driveTrainRobotDrive21.arcadeDrive(/*moveValue*/ motorPower, /*rotateValue*/ rotationPower); 
		    SmartDashboard.putNumber("RobotDriveStraightPIDOoutput Motor Output",motorPower);
		    SmartDashboard.putNumber("RobotDriveStraightPIDOoutput RotationPower", rotationPower);
	    	double leftPower; 
	    	double rightPower;
	    	
	    	// Reduce joystick power so can get full turning effect, and hopefully avoid a jerk in the rotation
	    	// if quickly taken off full throttle.
	    	if (motorPower + rotationPower > 1.0){
	    		motorPower = 1 - rotationPower;
	    	}
	    	
	    	leftPower = motorPower-rotationPower;
	    	rightPower = motorPower+rotationPower;
	    	robotDrive.tankDrive(leftPower, rightPower);

		}
		
		public  MotionControlPIDController createRotationPIDController(double targetAngle, double start, PIDOutput pidOutput) {
			
		    double     ramp =  30; //degrees
		    double maxspeed = 10.0*(360/60) ; //60/360 converts the first numbers which is in RPM to degrees/second
			
			final double Kp = 1d/200; // so at denominator off in the spin-Rate the power will reach the max
		    final double Ki = 0.0000;
		    final double Kd = 0.0;
		 
		    MotionControlPIDController localRotationSpeedPID;

		    MotionControlHelper rotationSpeedProfile; 
	        rotationSpeedProfile = new MotionControlHelper(targetAngle, ramp, maxspeed, start, (PIDSource) gyro, pidOutput);
	        localRotationSpeedPID = new MotionControlPIDController(Kp,Ki,Kd, rotationSpeedProfile );
	        localRotationSpeedPID.setOutputRange(-1.0, 1.0);
	        localRotationSpeedPID.enable();
		    return localRotationSpeedPID;
		}
		
		
	    private class WrapRotationPIDOutput implements PIDOutput {

	        private DriveStraightPIDOutput m_rotationPowerDestination;

	        public WrapRotationPIDOutput(DriveStraightPIDOutput rotationPowerDesintation) {
	            if (rotationPowerDesintation == null) {
	                throw new NullPointerException("Given rotationPowerDestination was null");
	            }
	            else{
	                m_rotationPowerDestination = rotationPowerDesintation;            	
	            }
	        }

			@Override
			public void pidWrite(double rotationPower) {
				this.m_rotationPowerDestination.setRotationPower(rotationPower);
			}

	    }



	}

