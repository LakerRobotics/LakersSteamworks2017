package org.usfirst.frc.team5053.robot;

import org.usfirst.frc.team5053.robot.RobotInterfaceMap.JoystickType;
import org.usfirst.frc.team5053.robot.Subsystems.DriveTrain;
import org.usfirst.frc.team5053.robot.Subsystems.DriveTrainMotionControl;
import org.usfirst.frc.team5053.robot.Subsystems.Indexer;
import org.usfirst.frc.team5053.robot.Subsystems.Intake;
import org.usfirst.frc.team5053.robot.Subsystems.LightSystem;
import org.usfirst.frc.team5053.robot.Subsystems.Mixer;
import org.usfirst.frc.team5053.robot.Subsystems.Scaler;
import org.usfirst.frc.team5053.robot.Subsystems.Shooter;
import org.usfirst.frc.team5053.robot.Subsystems.Utilities.GRIPVision;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
	private LightSystem m_LightSystem;
	
	//Vision declaration
	private UsbCamera m_Camera;
	private VisionThread m_VisionThread;
	private double centerX;
	private double previousVisionTurn;
	private boolean isVisionTurnRunning;
	private final Object m_ImgLock = new Object();
	
	//Shooter speeds
	private final double SHOOTER_SPEED	= .5;
	
	//Misc speeds
	private final double INTAKE_SPEED = .7;
	private final double MIXER_SPEED = .7;
	private final double INDEXER_SPEED = .7;
	private final double SCALER_SPEED = .7;

	private final int IMG_WIDTH		= 320;
	private final int IMG_HEIGHT 	= 240;
	private final int CAMERA_ANGLE 	= 52;
	
	//Misc Variables
	private int autonomousCase;
	private int teleopLightLoops;
	
	@Override
    public void robotInit()
    {
        /**
         * This function is run when the robot is first started up and should be
         * used for any initialization code.
         */
    	
    	// the RobotInterfaceMap detects the controller and set the key mapping accordingly
    	m_RobotInterface = new RobotInterfaceMap(JoystickType.XBOX, JoystickType.JOYSTICK);

    	m_RobotControllers = new RobotControllerMap();
    	m_RobotSensors = new RobotSensorMap();    	
    	
    	//Robot Subsystem Initialization
    	m_DriveTrain = new DriveTrainMotionControl(m_RobotControllers.getLeftDrive(), m_RobotControllers.getRightDrive(), m_RobotSensors.getLeftDriveEncoder(), m_RobotSensors.getRightDriveEncoder(), m_RobotSensors.getGyro());
    	m_Shooter = new Shooter(m_RobotControllers.getShooter(), m_RobotSensors.getShooterEncoder());
    	m_Intake = new Intake(m_RobotControllers.getIntake());
    	m_Indexer = new Indexer(m_RobotControllers.getIndexer());
    	m_Mixer = new Mixer(m_RobotControllers.getMixer());
    	m_Scaler = new Scaler(m_RobotControllers.getScaler());
    	
    	m_LightSystem = new LightSystem(m_RobotSensors.getRed(), m_RobotSensors.getBlue(), m_RobotSensors.getGreen(), DriverStation.getInstance().getAlliance());
    	m_LightSystem.setDefault();
    	
    	m_Camera = CameraServer.getInstance().startAutomaticCapture();
		
    	m_Camera.setExposureManual(1);
    	m_Camera.setFPS(30);
    	m_Camera.setBrightness(1);
        m_Camera.setResolution(IMG_WIDTH, IMG_HEIGHT);
    	
        centerX = 0.0;
    	autonomousCase = 0;
    	previousVisionTurn = 0;
    	teleopLightLoops = 0;
    	isVisionTurnRunning = false;
       
        m_VisionThread = new VisionThread(m_Camera, new GRIPVision(), pipeline -> {
        	SmartDashboard.putBoolean("Is empty", pipeline.filterContoursOutput().isEmpty());
            if (!pipeline.filterContoursOutput().isEmpty()) {
                Rect r = Imgproc.boundingRect(pipeline.filterContoursOutput().get(0));
                synchronized (m_ImgLock) {
                	centerX = r.x + (r.width / 2);
                    SmartDashboard.putNumber("Vision CenterX", centerX);
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
    		centerX = this.centerX;
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
    		centerX = this.centerX;
    	}
    	double turn = (centerX - (IMG_WIDTH / 2))/(IMG_WIDTH/2) * (CAMERA_ANGLE/2);
    	
        SmartDashboard.putString("Alliance", DriverStation.getInstance().getAlliance().toString());
    	
    	//Drivetrain
    	if(m_RobotInterface.GetDriverLeftBumper())
    	{
    		if(!(Math.abs(previousVisionTurn) == turn))
    		{
    			previousVisionTurn = turn;
        		visionAlign(turn);
    		}
    	}
    	else if(isVisionTurnRunning)
    	{
    		if(m_DriveTrain.isTurnPIDFinished())
        	{
        		isVisionTurnRunning = false;
        		m_DriveTrain.DisablePIDControl();
        	}
    	}
    	else if(!isVisionTurnRunning)
    	{
        	arcadeDrive();
    	}
    	m_DriveTrain.WriteDashboardData();
    	
    	//Shooter methods
    	shoot();
    	
    	//Other
    	//Indexer is not run here because AFAIK it should only be run when the shooter is ready to go
    	runIntake();
    	runIndexer();
    	runScaler();
    	
    	teleopLightLoops++;
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
    	m_DriveTrain.ArcadeDrive(m_RobotInterface.GetDriverLeftY(), m_RobotInterface.GetDriverRightX());
    }
    
    public void visionAlign(double turn) 
    {
    	SmartDashboard.putNumber("Vision Turn", turn);
    	
    	m_DriveTrain.TurnToAngle(turn);

		isVisionTurnRunning = true;	
    	if(m_DriveTrain.isTurnPIDFinished())
    	{
    		isVisionTurnRunning = false;
    		m_DriveTrain.DisablePIDControl();
    	}
    }
    
    //Shooter methods
    public void shoot()
    {
    	//TODO Determine buttons
    	if(m_RobotInterface.GetDriverRightBumper())
    	{
    		//TODO Add an encoder to the robot's shooter
    		m_Shooter.SetTalonOutput(SHOOTER_SPEED);
    		runIndexer();
    		
    		//Shoot
    		//Enable the PID Controller for the Shooter
    		/*m_Shooter.SetShooterSetpoint(SHOOTER_SPEED);
    	   	m_Shooter.EnablePID();
    		
        	if(m_Shooter.ShooterOnTarget())
        	{
        		//Shooter is ready to fire.
        		runIndexer();
        		
        		m_LightSystem.setRedState(false);
        		m_LightSystem.setBlueState(false);
        		m_LightSystem.setGreenState(true);
        	}
        	else
        	{
        		m_LightSystem.setDefault();
        	}*/
    	}
    	else
    	{
    		//STOP
    		//m_Shooter.DisablePID();
    		m_Shooter.SetTalonOutput(0);
    		//m_LightSystem.setDefault();
    	}
    }

    //Intake methods
    public void runIntake()
    {
	    if(m_RobotInterface.GetDriverA())
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
    	if(m_RobotInterface.GetDriverB())
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
    	if(m_RobotInterface.GetDriverY())
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
    	if(m_RobotInterface.GetDriverX())
 		{
        	m_Scaler.SetTalonOutput(SCALER_SPEED);
        	
        	if(teleopLightLoops <= 50)
        	{
        		m_LightSystem.setGreenState(true);
        		
        		m_LightSystem.setRedState(false);
        		m_LightSystem.setBlueState(false);
        	}
        	else if(teleopLightLoops > 50 && teleopLightLoops <= 100)
        	{
        		m_LightSystem.setRedState(true);
        		
        		m_LightSystem.setBlueState(false);
        		m_LightSystem.setGreenState(false);
        	}
        	else if(teleopLightLoops > 100)
        	{
        		m_LightSystem.setRedState(true);
        		
        		m_LightSystem.setBlueState(false);
        		m_LightSystem.setGreenState(false);
        		if(teleopLightLoops == 150)
        			teleopLightLoops = 0;
        	}
 		}
 	    else
 	    {
 	    	m_Scaler.SetTalonOutput(0);
 	    	m_LightSystem.setDefault();
 	    }
    }
    
    public void GetDashboardData()
    {
    	//Use this to retrieve values from the Driver Station
    	//e.g Which autonomous to use or processed image information.
    }
}