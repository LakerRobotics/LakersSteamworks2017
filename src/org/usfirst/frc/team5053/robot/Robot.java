package org.usfirst.frc.team5053.robot;

import org.usfirst.frc.team5053.robot.RobotInterfaceMap.joystickType;
import org.usfirst.frc.team5053.robot.Subsystems.Arm;
import org.usfirst.frc.team5053.robot.Subsystems.DriveTrain;
import org.usfirst.frc.team5053.robot.Subsystems.Intake;
import org.usfirst.frc.team5053.robot.Subsystems.Shooter;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 * 
 * STEAMWORKS 2017
 * Laker Robotics 
 * 
 */

public class Robot extends IterativeRobot
{

	/* Declare any and all variables here
	 *  Do not initialize until robotInit()
	 */
	
	//Robot Map Declaration
	RobotInterfaceMap m_RobotInterface;
	RobotControllerMap m_RobotControllers;
	RobotSensorMap m_RobotSensors;
	
	
	
	//Robot Subsystem Declaration
	DriveTrain m_DriveTrain;
	Arm m_Arm;
	Intake m_Intake;
	Shooter m_Shooter;
	//Arm setpoints
	final double ARM_NEUTRAL	= 0.209;
	final double ARM_AUTO		= 0.300;
	final double ARM_BALL		= 0.327;
	final double ARM_LOW		= 0.352;
	
	//Intake speeds
	final double INTAKE_FAST	= -2000;
	final double INTAKE_SLOW	= -1500;
	final double INTAKE_REVERSE =  2000;
	
	//Shooter speeds
	final double SHOOTER_FAST	= -380;
	final double SHOOTER_SLOW	= -360;
	final double SHOOTER_INTAKE	=  360;

    public void robotInit()
    {
        /**
         * This function is run when the robot is first started up and should be
         * used for any initialization code.
         */
    	
    	m_RobotInterface = new RobotInterfaceMap(joystickType.XBOX, joystickType.JOYSTICK);
    	//m_RobotInterface = new RobotInterfaceMap();
    	
    	m_RobotControllers = new RobotControllerMap();
    	m_RobotSensors = new RobotSensorMap();
    	
    	
    	
    	//Robot Subsystem Initialization
    	m_DriveTrain = new DriveTrain(m_RobotControllers.GetLeftDrive(), m_RobotControllers.GetRightDrive());
    	m_Arm = new Arm(m_RobotControllers.GetArm(), m_RobotSensors.GetArmPot());
    	
    	m_Shooter = new Shooter(m_RobotControllers.GetShooter(), m_RobotSensors.GetShooterEncoder());
    	m_Intake = new Intake(m_RobotControllers.GetIntake());
    }

    public void autonomousInit() 
    {
    	   /**
         * This function is called once when autonomous begins
         */
    }

    public void autonomousPeriodic()
    {

        /**
         * This function is called periodically during autonomous
         */
    }


    public void teleopPeriodic()
    {
        /**
         * This function is called periodically during operator control
         */
    	arcadeDrive();
    	
    	//Arm
    	if(m_RobotInterface.GetOperatorButton(1))
    	{
    		manualArmControl();
    	}
    	else
    	{
    		armSetpoints();
    	}

    	//Intake
    	intake();
    	
    	//Shooter
    	//shoot();
    	
    	//Update Dashboard Variables
    	GetDashboardData();
    	
    	
    }

    public void testPeriodic()
    {
        
        /**
         * This function is called periodically during test mode
         */
    }
    //Drivetrain
    public void arcadeDrive()
    {
    	m_DriveTrain.arcadeDrive(m_RobotInterface.GetDriverLeftY(), m_RobotInterface.GetDriverRightX());
    }
    
    //We don't have an arm... yet!
    public void manualArmControl() 
    {
    	//Gives the operator manual control over the arm if setpoints aren't working (sensor failure)
    	m_Arm.DisablePID();
        m_Arm.SetTalonOutput(-m_RobotInterface.GetOperatorJoystick().getRawAxis(1));
    }
    
    
    public void armSetpoints() 
    {	
    	//Gives the operator several different preset positions for an arm.
    	if(!m_Arm.isPIDEnabled())
    	{
    		m_Arm.SetTalonOutput(0);
    	}
    	
    	if(m_RobotInterface.GetOperatorButton(7))
    	{
        	m_Arm.EnablePID();
    		m_Arm.SetTargetPosition(ARM_NEUTRAL);
    	}
    	else if(m_RobotInterface.GetOperatorButton(9))
    	{
    		m_Arm.EnablePID();
    		m_Arm.SetTargetPosition(ARM_BALL);
    	}
    	else if(m_RobotInterface.GetOperatorButton(11))
    	{
    		m_Arm.EnablePID();
    		m_Arm.SetTargetPosition(ARM_LOW);
    	}
    }
    
    //Intake
    public void intake()
    {
	    if(m_RobotInterface.GetOperatorButton(8))
		{
			m_Intake.SetTalonOutput(INTAKE_FAST);
		}
		else if(m_RobotInterface.GetOperatorButton(10))
		{
			m_Intake.SetTalonOutput(INTAKE_SLOW);
			
		}
		else if(m_RobotInterface.GetOperatorButton(12))
		{
			m_Intake.SetTalonOutput(INTAKE_REVERSE);

		}
		else
		{
			m_Intake.SetTalonOutput(0);
		}
    }
    
    //Shooter
    public void shoot()
    {
    	
    	if(m_RobotInterface.GetOperatorButton(3))
    	{
    		//Fast Shot
    		//Enable the PID Controller for the Shooter
    		m_Shooter.SetShooterSetpoint(SHOOTER_FAST);
    	   	m_Shooter.EnablePID();
        	
        	
        	if(m_Shooter.ShooterOnTarget() && m_Shooter.ShooterOnTarget())
        	{
        		//Shooter is ready to fire.
        	}
    	}
    	else if(m_RobotInterface.GetOperatorButton(4))
    	{
    		//Slow Shot
    		//Enable the PID Controller for the Shooter
    		m_Shooter.SetShooterSetpoint(SHOOTER_SLOW);
    	   	m_Shooter.EnablePID();
        	
        	
        	if(m_Shooter.ShooterOnTarget() && m_Shooter.ShooterOnTarget())
        	{
        		//Shooter is ready to fire.
        	}
    	}
    	else
    	{
    		//STOP
    		m_Shooter.SetTalonOutput(0);
    	}
    }
    public void GetDashboardData()
    {
    	//Use this to retrieve values from the Driver Station
    	//e.g Which autonomous to use or processed image information.
    }
}