// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package org.usfirst.frc.team5053.robot.Subsystems.Utilities;

import edu.wpi.first.wpilibj.NamedSendable;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotDrive;

import edu.wpi.first.wpilibj.interfaces.Gyro;

//import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team5053.robot.Subsystems.Utilities.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team5053.robot.Subsystems.Utilities.MotionControlHelper;
import org.usfirst.frc.team5053.robot.Subsystems.Utilities.MotionControlPIDController;
import org.usfirst.frc.team5053.robot.Robot;
//import org.usfirst.frc.team5053.robot.Subsystems.Utilities.DriveStraightSpeedPIDOutput;
import org.usfirst.frc.team5053.robot.RobotSensorMap;


/**
 *
 */
public class MotionControllerDistance extends Command {
	// Tie this motion control into the robot by tieing int the Encoders and Gyro & DriveTrain
	
//	Gyro gyro = Robot.getRobotSensors().getGyro();
//	Encoder driveTrainLeftWheelEncoder = Robot.getRobotSensors().getLeftDrive();
//	Encoder driveTrainRightWheelEncoder = Robot.getRobotSensors().getRightDrive();
	RobotDrive robotDrive = Robot.getDriveTrain().getRobotDrive();

	public static Encoder m_LeftDrive;
	public static Encoder m_RightDrive;
	
	public  ADXRS450_Gyro m_Gyro;
	
	
	MotionControlPIDController motionControlHelper = null  ;
//TODO    private final PIDController drivePowerPID;
// Use these to get going:
// setSetpoint() -  Sets where the PID controller should move the system
//                  to
// enable() - Enables the PID controller.
double targetAngle = 0;	
double distance = 0; //Temp, will be set to an actual value below
MotionControlPIDController speedFollowerPID;
double targetTolerance = 5; //inch
double maxspeed = 0; //first number is Ft/sec the *12 changes it to in/sec
double ramp = 0; //inches distance to go from start to maxspeed and maxspeed to 0 at the end

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS
 
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR

        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);) {

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR

	//Auto generated parameter code will reappear each time we export from robot builder
	//comment out the excess code when you regenerate the code

	public MotionControllerDistance(double a_distance)
	{
		distance = -a_distance; //TODO we really should not have to negate the value
		maxspeed = 6 * 12;
		ramp = 7;
	}
	
	public MotionControllerDistance(double a_distance, double a_maxspeed)
	{
		distance = -a_distance; //TODO we really should not have to negate the value
    	maxspeed = a_maxspeed * 12;
    	ramp = 7;
	}

	public MotionControllerDistance(double a_distance, double a_maxspeed, double a_ramp)
	{
		distance = -a_distance;
		maxspeed = a_maxspeed * 12;
		ramp = a_ramp;
    	
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
        requires(Robot.getDriveTrain());

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
    }

    // Called just before this Command runs the first time
    public void initialize() {
    	m_Gyro = Robot.getRobotSensors().getGyro();
    	m_LeftDrive = Robot.getRobotSensors().getLeftDrive();
    	m_RightDrive = Robot.getRobotSensors().getRightDrive();
    	robotDrive = Robot.getDriveTrain().getRobotDrive();

//fromRudy        SmartDashboard.putNumber("Timeinit",this.timeSinceInitialized());
       	//RobotMap.gyroToUse.reset();//TODO don't reset the gyro.
    	targetAngle = m_Gyro.getAngle();	
    	m_LeftDrive.reset();
    	m_RightDrive.reset();
        double start = 0; //inches 
        //TODO make it read both and average
        m_RightDrive.setPIDSourceType(PIDSourceType.kRate);

        // Setup the motion control (i.e. how fast we are going as we move towards our destination and plus, rampUp/rampDown distances)
        // and use the driveStraight PIDOutput to pass along the speed we want to the PID Controller
        MotionControlHelper speedControl = new MotionControlHelper(distance, ramp, maxspeed, start,
        		m_RightDrive,//RobotMap.driveTrainRightWheelEncoder,
        		                                    new DriveStraightPIDOutput(m_Gyro, targetAngle));

        // setup the Pid to control how we follow the Speed
        final double Kp = 0.005;
        final double Ki = 0.00;
        final double Kd = 0.0;

    	speedFollowerPID = new MotionControlPIDController(Kp, Ki, Kd, speedControl );
    	speedFollowerPID.setAbsoluteTolerance(targetTolerance);
    	speedFollowerPID.setOutputRange(-1, 1);
    }

    // Called repeatedly when this Command is scheduled to run
    public void execute() {{
    	speedFollowerPID.enable();
//SmartDashboard.putData((NamedSendable) RobotMap.IMU);
    	
	   double angle = m_Gyro.getAngle(); // get current heading
       SmartDashboard.putNumber("MotionControllerDistance angle", angle);
       
       SmartDashboard.putNumber("MotionControllerDistance Left",m_LeftDrive.getDistance());
       SmartDashboard.putNumber("MotionControllerDistance Right",m_RightDrive.getDistance());
       SmartDashboard.putNumber("MotionControllerDistance Left Rate",m_LeftDrive.getRate());
//       SmartDashboard.putNumber("Left Target Rate",targetSpeed);
       System.out.println(//fromRudy "Time="+this.timeSinceInitialized()+
                         " encoderDist="+m_LeftDrive.getDistance()
//                         +" Left Target Rate="+targetSpeed);
);       
            

   }
    }

    // Make this return true when this Command no longer needs to run execute()
    public boolean isFinished() {
    	SmartDashboard.putString("MotionControllerDistance", "isFinished()");
  /*   if(speedFollowerPID.onTarget());
     {
    	 speedFollowerPID.disable();
     }
    	return speedFollowerPID.onTarget();
    	return false; */
    	
    	if(Math.abs(m_RightDrive.getDistance()-distance)<targetTolerance) { 
       		speedFollowerPID.disable();
//       		mcPID.getError() 
       		System.out.println("MotionControllerDistance Finished true");  
//20170208 because may not end at zero speed.       	  	robotDrive.tankDrive(0,0);
       		return  true;
       	}
       	else
       	{
               return false;
       	}

    }

    // Called once after isFinished returns true
    public void end() {
    	SmartDashboard.putString("autonForward", "end()");
    	speedFollowerPID.disable();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    public void interrupted() {
    	SmartDashboard.putString("autonForward", "interrupted()");
    	end();
      }
}
