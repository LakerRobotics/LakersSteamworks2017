package org.usfirst.frc.team5053.robot;

import org.usfirst.frc.team5053.robot.RobotInterfaceMap.joystickType;
import org.usfirst.frc.team5053.robot.Subsystems.Arm;
import org.usfirst.frc.team5053.robot.Subsystems.DriveTrain;
import org.usfirst.frc.team5053.robot.Subsystems.Intake;
import org.usfirst.frc.team5053.robot.Subsystems.Shooter;
import org.usfirst.frc.team5053.robot.Subsystems.Utilities.GRIPVision;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.vision.VisionPipeline;
import edu.wpi.first.wpilibj.vision.VisionRunner;
import edu.wpi.first.wpilibj.vision.VisionThread;
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
	
	int autonomousStep;
	int selectedAutonomous;
	int autoLoops;

	private final int IMG_WIDTH		= 320;
	private final int IMG_HEIGHT 	= 240;
	private final int CAMERA_ANGLE 	= 52;
	
	private VisionThread m_VisionThread;
	private double m_CenterX = 0.0;
	
	private final Object m_ImgLock = new Object();
	
	@Override
    public void robotInit()
    {
        /**
         * This function is run when the robot is first started up and should be
         * used for any initialization code.
         */
    	
    	// the RobotInterfaceMap detects the controller and set the key mapping accordingly
    	m_RobotInterface = new RobotInterfaceMap();
    	
    	
    	m_RobotControllers = new RobotControllerMap();
    	m_RobotSensors = new RobotSensorMap();
    	
    	
    	
    	//Robot Subsystem Initialization
    	m_DriveTrain = new DriveTrain(m_RobotControllers.GetLeftDrive(), m_RobotControllers.GetRightDrive());
    	m_Arm = new Arm(m_RobotControllers.GetArm(), m_RobotSensors.GetArmPot());
    	
    	m_Shooter = new Shooter(m_RobotControllers.GetShooter(), m_RobotSensors.GetShooterEncoder());
    	m_Intake = new Intake(m_RobotControllers.GetIntake());
    	
<<<<<<< HEAD
    	autonomousStep = 0;
    	selectedAutonomous = 0;
    	autoLoops = 0;
=======
    	UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();

    	camera.setExposureManual(1);
    	camera.setFPS(30);
    	camera.setBrightness(1);
        camera.setResolution(IMG_WIDTH, IMG_HEIGHT);
        
        m_VisionThread = new VisionThread(camera, new GRIPVision(), pipeline -> {
            if (!pipeline.filterContoursOutput().isEmpty()) {
                Rect r = Imgproc.boundingRect(pipeline.filterContoursOutput().get(0));
                synchronized (m_ImgLock) {
                    m_CenterX = r.x + (r.width / 2);
                    SmartDashboard.putNumber("Vision CenterX", m_CenterX);
                }
            }
        });
        m_VisionThread.start();
    	
>>>>>>> origin/master
    }

    public void autonomousInit() 
    {
    	 /**
         * This function is called once when autonomous begins
         */
    	
    	
    }

    public void autonomousPeriodic()
    {
    	
    	double centerX;
    	synchronized (m_ImgLock) {
    		centerX = this.m_CenterX;
    	}
    	double turn = (centerX - (IMG_WIDTH / 2))/(IMG_WIDTH/2) * (CAMERA_ANGLE/2);

    	SmartDashboard.putNumber("Vision Turn", turn);
    	//m_DriveTrain.arcadeDrive(-0.6, turn * 0.005);
    	
        /**
         * This function is called periodically during autonomous
         */
    	switch(selectedAutonomous)
    	{
    	case 0:
    		BasicAutonomous();
    		break;
    	}
    	autoLoops++;
    }


    public void teleopPeriodic()
    {
    	SmartDashboard.putNumber("Vision CenterX", m_CenterX);
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
    public void BasicAutonomous()
    {
    	switch(autonomousStep)
    	{
    	case 0:
    		m_DriveTrain.arcadeDrive(.25, 0.0);
    		autonomousStep++;
    		break;
    	case 1:
    		if(autoLoops >= 150) // 3 Seconds
    		{
    			m_DriveTrain.arcadeDrive(0.0, 0.0);
    			autonomousStep++;
    		}
    		break;    
    	case 2:
    		//Done
    		break;
    	}
    }
    public void GetDashboardData()
    {
    	//Use this to retrieve values from the Driver Station
    	//e.g Which autonomous to use or processed image information.
    }
}