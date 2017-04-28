package org.usfirst.frc.team5053.robot.Subsystems.Utilities;

import org.usfirst.frc.team5053.robot.Subsystems.DriveTrainMotionControl;

import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;

	import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

	/**
	 * 
	 * @author Rich Topolewski
	 * 
	 * Used to take the speed calculated from the PID control and pass it to the drive train, 
	 * and adjust the power difference between the wheels to drive on an arc of specified Radius 
	 *
	 */
	public class ArcMotionPIDOutput implements PIDOutput {
		//Gyro gyro = Robot.getRobotSensors().getGyro();
		
		// This is just a simple P control, Proportional control of the line follow
		// if we assume angle is in degrees and if we were off by 20 Degrees then we would want how much correction
		// for example id Kp is 0.025 at 20 degrees we would have 0.5 or half the power toward rotating the robot 

		private DriveTrainMotionControl m_RobotDrive;
		private double Kp;
		private PIDSource m_Gyro; 
		private double m_TargetAngle;
		private double m_RotationPower;
		private double m_ForwardPower;
		private double m_ArcRadius;
		MotionControlPIDController m_ArcRotationSpeedPID;

		public ArcMotionPIDOutput(DriveTrainMotionControl drive, PIDSource anglePIDSource, double arcRadius) {
			//SmartDashboard.putString("DriveSpinPIDOutput", "constructor called");
			m_RobotDrive 	= drive;
			m_Gyro 			= anglePIDSource	;
			m_TargetAngle 	= Double.MAX_VALUE;
			m_ArcRadius 	= arcRadius;
			
			Kp 				= 0d/20d; //0.025;//
			m_TargetAngle 	= 0.0d;
			m_RotationPower = 0.0d;
			m_ForwardPower  = 0.0d;
			
			double slowRotation 					= m_TargetAngle + 90;
			WrapArcPIDOutput wrappedArcPIDOutput 	=  new WrapArcPIDOutput(this);
			
			m_ArcRotationSpeedPID = createArcPIDController(m_TargetAngle, slowRotation, wrappedArcPIDOutput);
			
			//WrapRotationPIDInput  wrapRotationPIDInput = new WrapRotationPIDOutput(rotationPID, (PIDSource) m_gyro);
		}

		protected synchronized double getRotationPower() {
			return m_RotationPower;
		}


		protected synchronized void setRotationPower(double rotationPower) {
			m_RotationPower = rotationPower;
		}
		
		@Override
		public synchronized void pidWrite(double forwardPower) 
		{
		    //rotationPower
		   	//double rotationPower = 0;
		   	//RobotMap.driveTrainRobotDrive21.arcadeDrive(/*moveValue*/ motorPower, /*rotateValue*/ rotationPower);
			
			//store it away so rotation can be set appropriately
			m_ForwardPower = forwardPower;			
					 
			//Adjust the rotation power to be proportional to the average speed of the Robot to stay on the desired arch
			m_ArcRotationSpeedPID.setSetpoint(getTargetRotationalSpeed());
			
			SmartDashboard.putNumber("Arc - Target Rotational Speed", m_ArcRotationSpeedPID.getSetpoint());
	    	
		    double leftPower; 
	    	double rightPower;
	    	
	    	// Reduce forward power so can get full and even turning effect
	    	// also may help if quickly reduce from full throttle, to avoid a jerk in the rotation as the PID would convert from 1/2 to all all rotation power
	    	// .
	    	
	    	//m_RotationPower = 0;
	    	
	    	if (forwardPower + m_RotationPower > 1.0)
	    	{
	    		forwardPower = 1 - m_RotationPower;
	    	}
	    	
	    	SmartDashboard.putNumber("Arc - Rotational Power", m_RotationPower);
	    	SmartDashboard.putNumber("Arc - Forward Power", forwardPower);
	    	
	    	leftPower = m_ForwardPower - m_RotationPower;
	    	rightPower = m_ForwardPower + m_RotationPower;
	    	
	    	SmartDashboard.putNumber("Arc - Left Power Output", leftPower);
	    	SmartDashboard.putNumber("Arc - Right Power Output", rightPower);
	    	
	    	double multiplier = 1.0;
	    	
	    	m_RobotDrive.tankDrive(leftPower * multiplier, rightPower * multiplier);

		}
		
		private double getTargetRotationalSpeed()
		{
			//set the rotation based on the current speed of the robot.
			//arched-turn for a robot in a big circle of radius R and it seems the the rate of angler change just needs to be proportional to the speed:
			// RateAngularChange = 360*Speed/2pi*R,
			//		 where: Speed is the speed of the robot in ft/sec
			//		 pi is the constant pi
			//		 R is the Radius of the turn path we want the robot to take in ft.
			double speed = m_RobotDrive.GetAverageSpeed();
			double angularChangeSpeed = (speed * 360)/(2 * Math.PI * m_ArcRadius);
			return angularChangeSpeed;
		}	
		
		public  MotionControlPIDController createArcPIDController(double targetAngle, double startAngle, PIDOutput pidOutput) {
			
		    double ramp 	=  0; //degrees 20170208 this would be best to follow the Radius
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
	        
		    rotationSpeedProfile = new MotionControlHelper(targetAngle, ramp, maxspeed, startAngle, m_Gyro, pidOutput);// this is being overiden in this.pidWrite()
	        
		    //TODO have the targetAngle stop the adjustment from this.pidWrite(..)
	        
		    localRotationSpeedPID = new MotionControlPIDController(Kp,Ki,Kd, rotationSpeedProfile );
	        localRotationSpeedPID.setOutputRange(-1.0, 1.0);
	        localRotationSpeedPID.enable();
	        
		    return localRotationSpeedPID;
		}
		
		
	    public double getForwardPower() {
			return m_ForwardPower;
		}



	    
		private class WrapArcPIDOutput implements PIDOutput {

	        private ArcMotionPIDOutput m_RotationPowerDestination;

	        //Constructor
	        public WrapArcPIDOutput(ArcMotionPIDOutput rotationPowerDesintation) {
	            if (rotationPowerDesintation == null) 
	            {
	                throw new NullPointerException("Given rotationPowerDestination was null");
	            }
	            else
	            {
	            	m_RotationPowerDestination = rotationPowerDesintation;            	
	            }
	        }

			@Override
			public void pidWrite(double rotationPower) {
				this.m_RotationPowerDestination.setRotationPower(rotationPower);
			}
	    }
	}