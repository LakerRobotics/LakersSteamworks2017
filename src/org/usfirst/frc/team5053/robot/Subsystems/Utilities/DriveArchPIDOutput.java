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
	 * and adjust the power difference between the wheels to drive on an arch of specified Radius 
	 *
	 */
	public class DriveArchPIDOutput implements PIDOutput {
		RobotDrive robotDrive = Robot.getDriveTrain().getRobotDrive();
//		Gyro gyro = Robot.getRobotSensors().getGyro();
		
		// This is just a simple P control, Proportional control of the line follow
		// if we assume angle is in degrees and if we were off by 20 Degrees then we would want how much correction
		// for example id Kp is 0.025 at 20 degrees we would have 0.5 or half the power toward rotating the robot 
		double Kp = 0d/20d; //0.025;// 
		Gyro m_gyro; 
		double m_targetAngle = 0.0d;
		double rotationPower = 0.0d;
		private double forwardPower  = 0.0d;
		double m_archRadius    = 0.0d;
		MotionControlPIDController m_archRotationSpeedPID = null;

		public DriveArchPIDOutput(Gyro theGyro, double targetAngle, double archRadius) {
//		    SmartDashboard.putString("DriveSpinPIDOutput", "constructor called");
			m_gyro = theGyro;
			m_targetAngle = targetAngle;
			m_archRadius = archRadius;
			double slowRotation = m_targetAngle + 90;
			WrapArchPIDOutput wrappedArchPIDOutput =  new WrapArchPIDOutput(this);
			
			m_archRotationSpeedPID = createArchPIDController(m_targetAngle, slowRotation, wrappedArchPIDOutput);
			
			//WrapRotationPIDInput  wrapRotationPIDInput = new WrapRotationPIDOutput(rotationPID, (PIDSource) m_gyro);
		}

		protected synchronized double getRotationPower() {
			return rotationPower;
		}


		protected synchronized void setRotationPower(double rotationPower) {
			this.rotationPower = rotationPower;
		}


		@Override
		public synchronized void pidWrite(double a_forwardPower) {
		    //rotationPower
		   	//double rotationPower = 0;
		   	//RobotMap.driveTrainRobotDrive21.arcadeDrive(/*moveValue*/ motorPower, /*rotateValue*/ rotationPower);
			
			//store it away so rotation can be set appropriately
			forwardPower = a_forwardPower;
			
					 
			//if not at the desired total rotation angle, then adjust the rotation power
			//abs(m_archRotationPID.getMotionControlHelper().m_targetDistance - gyro.getAngle()) >
			if (m_archRotationSpeedPID.onTarget()){
				//we have turned to the correct angle so no rotation power except as needed based on the regular motion controlled angular adjustment				
			}else{
				//set the rotation based on the current speed of the robot.
				//arched-turn for a robot in a big circle of radius R and it seems the the rate of angler change just needs to be proportional to the speed:
				// RateAngularChange = 360*Speed/2pi*R,
				//		 where: Speed is the speed of the robot in ft/sec
				//		 pi is the constant pi
				//		 R is the Radius of the turn path we want the robot to take in ft.
				double speed = (Robot.getDriveTrain().GetLeftSpeed()+Robot.getDriveTrain().GetRightSpeed())/2;
				double angularChangeSpeed = (speed*360)/(2*3.14159265*m_archRadius);
				m_archRotationSpeedPID.setSetpoint(angularChangeSpeed);
			}
			
					
		    SmartDashboard.putNumber("RobotDriveStraightPIDOoutput Motor Output",forwardPower);
		    SmartDashboard.putNumber("RobotDriveStraightPIDOoutput RotationPower", rotationPower);
	    	double leftPower; 
	    	double rightPower;
	    	
	    	// Reduce forward power so can get full and even turning effect
	    	// also may help if quickly reduce from full throttle, to avoid a jerk in the rotation as the PID would convert from 1/2 to all all rotation power
	    	// .
	    	if (a_forwardPower + rotationPower > 1.0){
	    		a_forwardPower = 1 - rotationPower;
	    	}
	    	
	    	leftPower = forwardPower-rotationPower;
	    	rightPower = forwardPower+rotationPower;
	    	robotDrive.tankDrive(leftPower, rightPower);

		}
		
		public  MotionControlPIDController createArchPIDController(double targetAngle, double startAngle, PIDOutput pidOutput) {
			
		    double     ramp =  0; //degrees 20170208 this would be best to follow the Radius
		    double maxspeed = 10.0*(360/60) ; //60/360 converts the first numbers which is in RPM to degrees/second
			
			final double Kp = 1d/200; // so at denominator off in the spin-Rate the power will reach the max
		    final double Ki = 0.0000;
		    final double Kd = 0.0;
		 
		    MotionControlPIDController localRotationSpeedPID;

		    MotionControlHelper rotationSpeedProfile; 
		    //TODO have go a a rotation speed proportional to the average speed of the wheels
		    // Okay this had been adjusting the speedOfRotation based on how far off they were instead it needs to be adjusted based
		    // on the speed of the robot
		    // Could do this or could just be more generic and follow the speed of the robot, think this is simplier to start
	        rotationSpeedProfile = new MotionControlHelper(targetAngle, ramp, maxspeed, startAngle, (PIDSource) m_gyro, pidOutput);// this is being overiden in this.pidWrite()
	        //TODO have the targetAngle stop the adjustment from this.pidWrite(..)
	        localRotationSpeedPID = new MotionControlPIDController(Kp,Ki,Kd, rotationSpeedProfile );
	        localRotationSpeedPID.setOutputRange(-1.0, 1.0);
	        localRotationSpeedPID.enable();
		    return localRotationSpeedPID;
		}
		
		
	    public double getForwardPower() {
			return forwardPower;
		}



		private class WrapArchPIDOutput implements PIDOutput {

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
				this.m_rotationPowerDestination.setRotationPower(rotationPower);
			}

	    }



	}

