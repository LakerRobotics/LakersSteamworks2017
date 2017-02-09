package org.usfirst.frc.team5053.robot;

import org.usfirst.frc.team5053.robot.Subsystems.DriveTrain;
import org.usfirst.frc.team5053.robot.Subsystems.DriveTrainMotionControl;
import org.usfirst.frc.team5053.robot.Subsystems.Indexer;
import org.usfirst.frc.team5053.robot.Subsystems.Intake;
import org.usfirst.frc.team5053.robot.Subsystems.Mixer;
import org.usfirst.frc.team5053.robot.Subsystems.Scaler;
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
	private RobotInterfaceMap m_RobotInterface;
	private RobotControllerMap m_RobotControllers;
	private RobotSensorMap m_RobotSensors;
	
	//Robot Subsystem Declaration
	//DriveTrain m_DriveTrain;
	private DriveTrainMotionControl m_DriveTrain;
	private Intake m_Intake;
	private Shooter m_Shooter;
	private Indexer m_Indexer;
	private Scaler m_Scaler;
	private Mixer m_Mixer;
	
	//Shooter speeds
	private final double SHOOTER_FAST	= -380;
	private final double SHOOTER_SLOW	= -360;
	
	//Misc speeds
	private final double INTAKE_SPEED = 360;
	private final double MIXER_SPEED = 360;
	private final double INDEXER_SPEED = 360;
	private final double SCALER_SPEED = 360;

	private final int IMG_WIDTH		= 320;
	private final int IMG_HEIGHT 	= 240;
	private final int CAMERA_ANGLE 	= 52;
	
	private VisionThread m_VisionThread;
	private double m_CenterX = 0.0;
	
	private final Object m_ImgLock = new Object();
	
	private int autonomousCase;
	private boolean isVisionTurnRunning = false;
	
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
    	m_DriveTrain = new DriveTrainMotionControl(m_RobotControllers.getLeftDrive(), m_RobotControllers.getRightDrive(), m_RobotSensors.getLeftDriveEncoder(), m_RobotSensors.getRightDriveEncoder(), m_RobotSensors.getGyro());
    	m_Shooter = new Shooter(m_RobotControllers.getShooter(), m_RobotSensors.getShooterEncoder());
    	m_Intake = new Intake(m_RobotControllers.getIntake());
    	m_Indexer = new Indexer(m_RobotControllers.getIndexer());
    	m_Mixer = new Mixer(m_RobotControllers.getMixer());
    	m_Scaler = new Scaler(m_RobotControllers.getScaler());
    	
    	autonomousCase = 0;
    	
    	UsbCamera camera = CameraServer.getInstance().startAutomaticCapture("Shooter Cam", 0);
    	UsbCamera backCamera = CameraServer.getInstance().startAutomaticCapture("Peg Camera", 1);
    	SmartDashboard.putBoolean("Is second cam null?", backCamera == null);
    	
    	camera.setExposureManual(1);
    	camera.setFPS(30);
    	camera.setBrightness(1);
        camera.setResolution(IMG_WIDTH, IMG_HEIGHT);
        
        m_VisionThread = new VisionThread(camera, new GRIPVision(), pipeline -> {
        	SmartDashboard.putBoolean("Is empty", pipeline.filterContoursOutput().isEmpty());
            if (!pipeline.filterContoursOutput().isEmpty()) {
                Rect r = Imgproc.boundingRect(pipeline.filterContoursOutput().get(0));
                synchronized (m_ImgLock) {
                    m_CenterX = r.x + (r.width / 2);
                    SmartDashboard.putNumber("Vision CenterX", m_CenterX);
                }
            }
        });
        m_VisionThread.start();
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
    	
    	double centerX;
    	synchronized (m_ImgLock) {
    		centerX = this.m_CenterX;
    	}
    	double turn = (centerX - (IMG_WIDTH / 2))/(IMG_WIDTH/2) * (CAMERA_ANGLE/2);

    	SmartDashboard.putNumber("Vision Turn", turn);
    	//m_DriveTrain.arcadeDrive(-0.6, turn * 0.005);
        
    	switch(autonomousCase)
    	{
    	case 0:
    		m_DriveTrain.ResetEncoders();
    		m_DriveTrain.ResetGyro();
    		//Magic Numbers
    		m_DriveTrain.DriveDistance(50, 10, 7);
    		m_DriveTrain.WriteDashboardData();
    		autonomousCase++;
    		System.out.println("Incrementing Case");
    		break;
    	case 1:
    		m_DriveTrain.WriteDashboardData();
    		if(m_DriveTrain.isStraightPIDFinished())
    		{
    			autonomousCase++;
    		}
    		break;
    	}
    }

    public void teleopPeriodic()
    {
    	/**
         * This function is called periodically during operator control
         */
    	
    	GetDashboardData();
    	
    	double centerX;
    	synchronized (m_ImgLock) {
    		centerX = this.m_CenterX;
    	}
    	double turn = (centerX - (IMG_WIDTH / 2))/(IMG_WIDTH/2) * (CAMERA_ANGLE/2);
    	SmartDashboard.putNumber("Teleop CenterX", m_CenterX);
    	SmartDashboard.putNumber("Teleop Turn", turn);
    	//Drivetrain
    	if(m_RobotInterface.GetDriverB() || isVisionTurnRunning)
    	{
    		visionAlign(turn);
    	}
    	else if(!isVisionTurnRunning)
    	{
        	arcadeDrive();
    	}
    	m_DriveTrain.WriteDashboardData();
    	
    	//Other
    	//Indexer is not run here because AFAIK it should only be run when the shooter is ready to go
    	//Is the mixer part of the intake system?
    	runIntake();
    	runMixer();
    	runScaler();
    }

    public void testPeriodic()
    {  
        /**
         * This function is called periodically during test mode
         */
    }
    
    //Drivetrain methods
    public void arcadeDrive()
    {
    	m_DriveTrain.ArcadeDrive(-m_RobotInterface.GetDriverLeftY(), -m_RobotInterface.GetDriverRightX());
    }
    
    public void visionAlign(double turn) 
    {
    	if(!m_DriveTrain.isPIDRunning && !isVisionTurnRunning)
    	{
        	SmartDashboard.putNumber("Vision Turn", turn);
        	
        	m_DriveTrain.TurnToAngle(turn);

    		isVisionTurnRunning = true;
    	}    	
    	else if(m_DriveTrain.isTurnPIDFinished())
    	{
    		isVisionTurnRunning = false;
    		m_DriveTrain.DisablePIDControl();
    	}
    }
    
    //Shooter methods
    public void shoot()
    {
    	//TODO Determine buttons
    	if(m_RobotInterface.GetOperatorButton(3))
    	{
    		//Fast Shot
    		//Enable the PID Controller for the Shooter
    		m_Shooter.SetShooterSetpoint(SHOOTER_FAST);
    	   	m_Shooter.EnablePID();
        	
        	
        	if(m_Shooter.ShooterOnTarget() && m_Shooter.ShooterOnTarget())
        	{
        		//Shooter is ready to fire.
        		runIndexer();
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
        		runIndexer();
        	}
    	}
    	else
    	{
    		//STOP
    		m_Shooter.SetTalonOutput(0);
    	}
    }

    //Intake methods
    public void runIntake()
    {
    	//TODO Determine button
	    if(m_RobotInterface.GetOperatorButton(8))
		{
			m_Intake.SetTalonOutput(INTAKE_SPEED);
		}
	    else
	    {
	    	m_Intake.SetTalonOutput(0);
	    }
    }
    
    //Indexer methods
    public void runIndexer() 
    {
    	//TODO Determine button
    	if(m_RobotInterface.GetOperatorButton(8))
 		{
    		m_Indexer.SetTalonOutput(INDEXER_SPEED);
 		}
 	    else
 	    {
 	    	m_Indexer.SetTalonOutput(0);
 	    }
    }
    
    //Mixer methods
    public void runMixer() 
    {
    	//TODO Determine button
    	if(m_RobotInterface.GetOperatorButton(8))
 		{
    	    m_Mixer.SetTalonOutput(MIXER_SPEED);
 		}
 	    else
 	    {
 	    	m_Mixer.SetTalonOutput(0);
 	    }
    }
    
    //Scaler methods
    public void runScaler() 
    {
    	//TODO Determine button
    	if(m_RobotInterface.GetOperatorButton(8))
 		{
        	m_Scaler.SetTalonOutput(SCALER_SPEED);
 		}
 	    else
 	    {
 	    	m_Scaler.SetTalonOutput(0);
 	    }
    }
    
    public void GetDashboardData()
    {
    	//Use this to retrieve values from the Driver Station
    	//e.g Which autonomous to use or processed image information.
    }
}