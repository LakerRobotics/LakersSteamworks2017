package org.usfirst.frc.team5053.robot;

import org.usfirst.frc.team5053.robot.RobotInterfaceMap.JoystickType;
import org.usfirst.frc.team5053.robot.Sensors.LidarLite;
import org.usfirst.frc.team5053.robot.Subsystems.DriveTrain;
import org.usfirst.frc.team5053.robot.Subsystems.DriveTrainMotionControl;
import org.usfirst.frc.team5053.robot.Subsystems.Indexer;
import org.usfirst.frc.team5053.robot.Subsystems.Intake;
import org.usfirst.frc.team5053.robot.Subsystems.LightSystem;
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
	private LightSystem m_LightSystem;
	private LidarLite m_Lidar;
	
	//Vision declaration
	private UsbCamera m_Camera;
	private VisionThread m_VisionThread;
	private double centerX;
	private double previousVisionTurn;
	private boolean isVisionTurnRunning;
	private final Object m_ImgLock = new Object();
	
	//Subsystem constants
	private final double DEFAULT_SHOOTER_SPEED	= .7;
	private final double DEFAULT_SHOOTER_RATE = 1000;
	private final double INTAKE_SPEED = .7;
	private final double MIXER_SPEED = .7;
	private final double INDEXER_SPEED = .7;
	private final double SCALER_SPEED = -1.0;

	//Vision constants
	private final int IMG_WIDTH		= 320;
	private final int IMG_HEIGHT 	= 240;
	private final int CAMERA_ANGLE 	= 52;
	
	//Misc Variables
	private int autonomousCase;
	private int autonomousWait;
	private int allianceSide;
	private int teleopLightLoops;
	private double shooterRPM;
	
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
    	m_Scaler = new Scaler(m_RobotControllers.getScaler());
    	
    	m_LightSystem = new LightSystem(m_RobotSensors.getRed(), m_RobotSensors.getBlue(), m_RobotSensors.getGreen(), DriverStation.getInstance().getAlliance());
    	m_LightSystem.setDefault();
    	m_Lidar = m_RobotSensors.getLidar();
    	m_Lidar.start();
    	
    	m_Camera = CameraServer.getInstance().startAutomaticCapture();
		
    	m_Camera.setExposureManual(1);
    	m_Camera.setFPS(30);
    	m_Camera.setBrightness(1);
        m_Camera.setResolution(IMG_WIDTH, IMG_HEIGHT);
    	
        centerX = 0.0;
    	autonomousCase = 0;
    	switch(DriverStation.getInstance().getAlliance())
    	{
    	case Red:
    		allianceSide = 1;
    		break;
    	case Blue:
    		allianceSide = -1;
    		break;
    	}
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
    	autonomousCase = 0;
    }

    public void autonomousPeriodic()
    {
    	/**
         * This function is called periodically during autonomous
         */
    	
    	double centerX;
    	double distanceToBoilerTarget;
    	synchronized (m_ImgLock) {
    		centerX = this.centerX;
    	}
    	double turn = (centerX - (IMG_WIDTH / 2))/(IMG_WIDTH/2) * (CAMERA_ANGLE/2);

    	SmartDashboard.putNumber("Vision Turn", turn);
    	
    	switch(SmartDashboard.getInt("autonRoutine", 0))
    	{
    	default:
    		autonRightGear();
    		break;
    	}
    	
		m_DriveTrain.WriteDashboardData();
		m_Shooter.WriteDashboardData();
    }
    
    public void autonLeftGearShoot(double visionTurn)
    {
    	switch(autonomousCase)
    	{
    	case 0: //Drive out from wall
    		m_DriveTrain.ResetEncoders();
    		m_DriveTrain.ResetGyro();
    		m_DriveTrain.DriveDistance(60 /*Distance to peg-start intersection*/, 4, 2);
    		autonomousCase++;
    		break;
    	case 1: //Turn to face the gear peg
    		if(m_DriveTrain.isStraightPIDFinished())
    		{
    			m_DriveTrain.ResetEncoders();
        		m_DriveTrain.ResetGyro();
        		m_DriveTrain.TurnToAngle(55*allianceSide/*angle to face the boiler*/);
    			autonomousCase++;
    		}
    		break;
    	case 2: //Engage Peg
    		if(m_DriveTrain.isTurnPIDFinished())
    		{
    			m_DriveTrain.ResetEncoders();
    			m_DriveTrain.ResetGyro();
    			m_DriveTrain.DriveDistance(80/*distance to the gear peg*/, 4, 2);
    			autonomousCase++;
    		}
    		break;
    	case 3: //Disengage Peg
    		if(m_DriveTrain.isStraightPIDFinished())
    		{
    			autonomousWait++;
    			if(autonomousWait >= 150)
    			{
        			m_DriveTrain.ResetEncoders();
            		m_DriveTrain.ResetGyro();
            		m_DriveTrain.DriveDistance(-80/*distance back to the intersection*/, 4, 2);
            		autonomousWait = 0;
        			autonomousCase++;
    			}
    		}
    		break;
    	case 4: //Turn to face the boiler
    		if(m_DriveTrain.isStraightPIDFinished())
    		{
        		m_DriveTrain.ResetEncoders();
        		m_DriveTrain.ResetGyro();
        		m_DriveTrain.TurnToAngle(120*allianceSide/*Turns the robot around so the high goal is within view for vision targeting*/);
        		autonomousCase++;
    		}
    		break;
    	case 5: //Vision align to high boiler
    		if(m_DriveTrain.isTurnPIDFinished())
    		{
    			if(!isVisionTurnRunning)
    			{
        			m_DriveTrain.ResetEncoders();
        			m_DriveTrain.ResetGyro();
    			}
        		if(!(Math.abs(previousVisionTurn) == visionTurn))
        		{
        			previousVisionTurn = visionTurn;
            		visionAlign(visionTurn /*Vision turn aligns to the reflective tape on the high goal*/);
        		}
        		if(!isVisionTurnRunning)
        		{
        			autonomousCase++;
        		}
    		}
    		break;
    	case 6: //Shoot
			m_DriveTrain.ResetEncoders();
			m_DriveTrain.ResetGyro();
			m_Shooter.SetShooterSetpoint(calculateShooterRPM()/*RPM calculated using lidar sensor*/);
			if(m_Shooter.ShooterOnTarget())
			{
				m_Indexer.SetTalonOutput(INDEXER_SPEED);
			}
    		break;
		default:
			break;
    	}
    }
    
    public void autonCenterGearShoot(double visionTurn)
    {
    	switch(autonomousCase)
    	{
    	case 0: //Drive out from wall and engage gear peg
    		m_DriveTrain.ResetEncoders();
    		m_DriveTrain.ResetGyro();
    		m_DriveTrain.DriveDistance(75 /*Distance to engage the gear peg*/, 10, 1);
    		autonomousCase++;
    		break;
    	case 1: //Disengage gear peg
    		if(m_DriveTrain.isStraightPIDFinished())
    		{
    			autonomousWait++;
    			if(autonomousWait >= 150)
				{
					m_DriveTrain.ResetEncoders();
		    		m_DriveTrain.ResetGyro();
		    		m_DriveTrain.DriveDistance(-24/*Distance to disengage gear peg*/, 10, 1);
					autonomousCase++;
    			}
    		}
    		break;
    	case 2: // Turn to face the boiler
    		if(m_DriveTrain.isStraightPIDFinished())
    		{
    			m_DriveTrain.ResetEncoders();
    			m_DriveTrain.ResetGyro();
    			m_DriveTrain.TurnToAngle(-115*allianceSide/*Angle to face the boiler*/);
    			autonomousCase++;
    		}
    		break;
    	case 3: //Vision align to high goal
    		if(m_DriveTrain.isTurnPIDFinished())
    		{
    			if(!isVisionTurnRunning)
    			{
        			m_DriveTrain.ResetEncoders();
        			m_DriveTrain.ResetGyro();
    			}
    			if(!(Math.abs(previousVisionTurn) == visionTurn))
        		{
        			previousVisionTurn = visionTurn;
            		visionAlign(visionTurn /*Vision turn aligns to the reflective tape on the high goal*/);
        		}
        		if(!isVisionTurnRunning)
        		{
        			autonomousCase++;
        		}
    		}
    	case 4: //Shoot
			m_DriveTrain.ResetEncoders();
			m_DriveTrain.ResetGyro();
			autonomousCase++;
    	}
    }
    
    public void autonRightGear()
	{
		switch(autonomousCase)
    	{
    	case 0: //Drive out from wall
    		m_DriveTrain.ResetEncoders();
    		m_DriveTrain.ResetGyro();
    		m_DriveTrain.DriveDistance(60 /*Distance to peg-start intersection*/, 4, 2);
    		autonomousCase++;
    		break;
    	case 1: //Turn to face the gear peg
    		if(m_DriveTrain.isStraightPIDFinished())
    		{
    			m_DriveTrain.ResetEncoders();
        		m_DriveTrain.ResetGyro();
        		m_DriveTrain.TurnToAngle(-55*allianceSide/*Angle to face the gear peg*/);
    			autonomousCase++;
    		}
    		break;
    	case 2: //Engage Peg
    		if(m_DriveTrain.isTurnPIDFinished())
    		{
    			m_DriveTrain.ResetEncoders();
    			m_DriveTrain.ResetGyro();
    			m_DriveTrain.DriveDistance(85/*Distance to gear peg*/, 4, 2);
    			autonomousCase++;
    		}
    		break;
    	case 3: //Disengage Peg
    		if(m_DriveTrain.isStraightPIDFinished())
    		{
    			autonomousWait++;
    			if(autonomousWait >= 150)
    			{
        			m_DriveTrain.ResetEncoders();
            		m_DriveTrain.ResetGyro();
            		m_DriveTrain.DriveDistance(-24/*Distance to disengage the peg*/, 4, 2);
            		autonomousWait = 0;
        			autonomousCase++;
    			}
    		}
    		break;
    	case 4:
    		if(m_DriveTrain.isStraightPIDFinished())
    		{
    			m_DriveTrain.ResetEncoders();
    			m_DriveTrain.ResetGyro();
    			autonomousCase++;
    		}
    	}
    }
    
    
    
    
    public void teleopPeriodic()
    {
    	/**
         * This function is called periodically during operator control
         */
    	runShooter();
    	runIndexer();
    	
    	if(autonomousCase != 0)
    	{
    		autonomousCase = 0;
    		m_DriveTrain.DisablePIDControl();
    	}
    	
    	GetDashboardData();
    	WriteDashboardData();
    	
    	double centerX;
    	synchronized (m_ImgLock) {
    		centerX = this.centerX;
    	}
    	double turn = (centerX - (IMG_WIDTH / 2))/(IMG_WIDTH/2) * (CAMERA_ANGLE/2);
    	
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
    	runShooter();
    	
    	//Other
    	runIntake();
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
    public void runShooter()
    {
    	GetDashboardData();
    //	if(m_RobotInterface.GetDriverRightBumper())
    	if(false)
    	{
    		//Enable the PID controller for the shooter using a rate calculated using our lidar sensor
    		m_Shooter.SetShooterSetpoint(calculateShooterRPM());

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
        	}
    	}
    	else if(m_RobotInterface.GetDriverY())
    	{
    		//Enable the PID controller for the shooter using a rate calculated read from the dashboard
    		System.out.println("Dashboard Shooter RPM: " + Double.toString(shooterRPM));
//    		m_Shooter.SetShooterSetpoint(shooterRPM);
//    	   	m_Shooter.EnablePID();
    		m_Shooter.SetTalonOutput(shooterRPM);

			SmartDashboard.putBoolean("Shooter on target", m_Shooter.ShooterOnTarget());
			
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
        	}
    	}
    	else
    	{
    		//STOP
    		m_Shooter.SetTalonOutput(0);
    		m_Shooter.DisablePID();
    		m_LightSystem.setDefault();
    	}
    }
    
    public double calculateShooterRPM()
    {
    	//y = 11.815x2 + 140.58x + 1348.4 
		double distanceFt = m_Lidar.getDistanceFt();
		double shooterRate = (11.815*(distanceFt*distanceFt) + 140.58*distanceFt+1348.4);
		System.out.println("Calculated Shooter RPM: " + Double.toString(shooterRate));
		return shooterRate;
    }

    //Intake methods
    public void runIntake()
    {
	    if(m_RobotInterface.GetOperatorButton(1))
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
    
    //Scaler methods
    public void runScaler() 
    {
    	//TODO Determine button
    	if(m_RobotInterface.GetOperatorButton(2))
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
    	shooterRPM = SmartDashboard.getNumber("shooterRPM", DEFAULT_SHOOTER_RATE);
    	//Use this to retrieve values from the Driver Station
    	//e.g Which autonomous to use or processed image information.
    }
    public void WriteDashboardData()
    {
    	m_DriveTrain.WriteDashboardData();
    	m_Shooter.WriteDashboardData();
    	SmartDashboard.putNumber("lidar", m_Lidar.getDistanceFt());
    }
}