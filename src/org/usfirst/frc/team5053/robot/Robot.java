package org.usfirst.frc.team5053.robot;

import org.usfirst.frc.team5053.robot.RobotInterfaceMap.JoystickType;
import org.usfirst.frc.team5053.robot.Sensors.LidarLite;
import org.usfirst.frc.team5053.robot.Subsystems.DriveTrainMotionControl;
import org.usfirst.frc.team5053.robot.Subsystems.Indexer;
import org.usfirst.frc.team5053.robot.Subsystems.Intake;
import org.usfirst.frc.team5053.robot.Subsystems.LightSystem;
import org.usfirst.frc.team5053.robot.Subsystems.Scaler;
import org.usfirst.frc.team5053.robot.Subsystems.Shooter;
//import org.usfirst.frc.team5053.robot.Subsystems.Utilities.GRIPVision;
//import org.opencv.core.Rect;
//import org.opencv.imgproc.Imgproc;

//import edu.wpi.cscore.UsbCamera;
//import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.NamedSendable;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
//import edu.wpi.first.wpilibj.vision.VisionThread;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.vision.USBCamera;


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
	private DriveTrainMotionControl m_DriveTrain;
	private Intake m_Intake;
	private Shooter m_Shooter;
	private Indexer m_Indexer;
	private Scaler m_Scaler;
	private LightSystem m_LightSystem;
	private LidarLite m_Lidar;
	
	//Vision declaration
	//private USBCamera m_Camera;
	//private VisionThread m_VisionThread;
	private double centerX;
	private double previousVisionTurn;
	private boolean isVisionTurnRunning;
	private final Object m_ImgLock = new Object();
	
	//Subsystem constants
	private final double DEFAULT_SHOOTER_SPEED	= .68;
	private final double DEFAULT_SHOOTER_RATE = 540;
	private final double INTAKE_SPEED = .7;
	private final double INDEXER_SPEED = 1;
	private final double SCALER_SPEED = -.85;

	//Vision constants
	private final int IMG_WIDTH		= 320;
	private final int IMG_HEIGHT 	= 240;
	private final int CAMERA_ANGLE 	= 52;
	
	//Autonomous variables
	private int autonomousRoutine;
	private int autonomousCase;
	private int autonomousWait;
	private boolean autonomousShoot;
	private int allianceSide;
	
	// Diagnostic variables
	private NetworkTable m_NetworkTable;
	private double driveTrainDiagnosticPower;
	private double[] diagnosticLeftRate;
	private double[] diagnosticRightRate;
	private double[] diagnosticPowerSent;
	private int arrIndex;
	
	//Misc variables
	private int teleopLightLoops;
	private double shooterRPM;
	private double shooterRPMBF;
	
	@Override
    public void robotInit()
    {
        /**
         * This function is run when the robot is first started up and should be
         * used for any initialization code.
         */
    	
    	m_RobotInterface = new RobotInterfaceMap(JoystickType.XBOX, JoystickType.JOYSTICK);
    	m_RobotControllers = new RobotControllerMap();
    	m_RobotSensors = new RobotSensorMap();    	
    	
    	//Robot Subsystem Initialization
    	m_DriveTrain = new DriveTrainMotionControl(m_RobotControllers.getLeftDrive(), m_RobotControllers.getRightDrive(), m_RobotSensors.getLeftDriveEncoder(), m_RobotSensors.getRightDriveEncoder(), m_RobotSensors.getGyro());
    	m_Shooter = new Shooter(m_RobotControllers.getShooter());//Encoder connected directly to the motor control so Robo Rio dosn't define the encoder since it knows nothing about it directly
    	m_Intake = new Intake(m_RobotControllers.getIntake());
    	m_Indexer = new Indexer(m_RobotControllers.getIndexer());
    	m_Scaler = new Scaler(m_RobotControllers.getScaler());
    	
    	//Misc Subsystem Initialization
    	m_LightSystem = new LightSystem(m_RobotSensors.getRed(), m_RobotSensors.getBlue(), m_RobotSensors.getGreen(), DriverStation.getInstance().getAlliance());
    	m_LightSystem.setDefault();
    	m_Lidar = m_RobotSensors.getLidar();
    	System.out.println("Lidar started: " + m_Lidar.getDistanceCm());
    	m_Lidar.start();
    	
    	shooterRPM = DEFAULT_SHOOTER_SPEED;
    	shooterRPMBF = DEFAULT_SHOOTER_RATE;
    	
    	// Diagnostic variable initialization
    	m_NetworkTable =  NetworkTable.getTable("SmartDashboard");
    	driveTrainDiagnosticPower = 0;
    	diagnosticLeftRate = new double[202];
    	diagnosticRightRate = new double[202];
    	diagnosticPowerSent = new double[202];
    	arrIndex = 0;
    	
    	//Camera Initialization
    	//CameraServer.getInstance().startAutomaticCapture();
    	//m_Camera.startCapture();
    	//m_Camera.setExposureManual(1);
    	//m_Camera.setFPS(30);
    	//m_Camera.setBrightness(1);
        //m_Camera.setResolution(IMG_WIDTH, IMG_HEIGHT);
    	
        centerX = (IMG_WIDTH/2);
    	previousVisionTurn = 0;
    	teleopLightLoops = 0;
    	isVisionTurnRunning = false;
       
        /*m_VisionThread = new VisionThread(m_Camera, new GRIPVision(), pipeline -> {
        	SmartDashboard.putBoolean("Is empty", pipeline.filterContoursOutput().isEmpty());
            if (!pipeline.filterContoursOutput().isEmpty()) {
                Rect r = Imgproc.boundingRect(pipeline.filterContoursOutput().get(0));
                synchronized (m_ImgLock) {
                	centerX = r.x + (r.width / 2);
                    SmartDashboard.putNumber("Vision CenterX", centerX);
                }
            }
        });
        m_VisionThread.start();*/
    }

    public void autonomousInit() 
    {
    	 /**
         * This function is called once when autonomous begins
         */
    	
    	autonomousRoutine = (int) SmartDashboard.getNumber("autonRoutine", 0);
    	autonomousCase = 0;
    	autonomousWait = 0;
    	autonomousShoot = SmartDashboard.getBoolean("shoot", true);
    	
    	switch(DriverStation.getInstance().getAlliance())
    	{
    	case Red:
    		allianceSide = -1;
    		break;
    	case Blue:
    		allianceSide = 1;
    		break;
		default:
    		allianceSide = 1;
    	}
    	
    	m_Shooter.EnablePID();
    }

    public void autonomousPeriodic()
    {
		
    	/**
         * This function is called periodically during autonomous
         */
    	
    	/*double centerX;
    	//Updates our vision angle
    	synchronized (m_ImgLock) {
    		centerX = this.centerX;
    	}
    	double turn = (centerX - (IMG_WIDTH / 2))/(IMG_WIDTH/2) * (CAMERA_ANGLE/2);

    	SmartDashboard.putNumber("Vision Turn", turn);*/
    	
    	double turn = 0;
    	
    	switch(autonomousRoutine)
    	{
    	case 0: // NO AUTON
    		break;
    	case 1: // CENTER
    		autonCenter(turn);
    		break;
    	case 2: // FEEDER
    		autonFeederSide(turn);
    		break;
    	case 3: // BOILER
    		autonBoilerSide(turn);
    		break;
    	case 4: // HOPPER
    		autonHopper(turn);
    		break;
    	case 5: //ARC FEEDER
    		break;
    	case 6: //ARC BOILER
    		break;
    	case 7: //ARC HOPPER
    		break;
    	case 8: // SENSORLESS CENTER
    		dumbAutonRoutine();
    		break;
    	case 9: // DEBUG
			diagnosticTest();
			break;
		default: // NO AUTON
			break;
    	}
    	
    	GetDashboardData();
    	WriteDashboardData();
    	
    	autonomousWait++;
    }
    
    public void diagnosticTest()
    {
    	m_DriveTrain.arcadeDrive(driveTrainDiagnosticPower/100, 0);
    	
    	switch(autonomousCase)
    	{
    	case 0:
    		if(autonomousWait >= 10)
        	{
        		if(driveTrainDiagnosticPower <= 99)
        		{
            		diagnosticPowerSent[arrIndex] = driveTrainDiagnosticPower/100;
            		diagnosticLeftRate[arrIndex] = m_DriveTrain.GetLeftSpeed();
            		diagnosticRightRate[arrIndex] = m_DriveTrain.GetRightSpeed();
            		
            		m_NetworkTable.putNumberArray("diagnosticLeftRate", diagnosticLeftRate);
        			m_NetworkTable.putNumberArray("diagnosticRightRate", diagnosticRightRate);
        			m_NetworkTable.putNumberArray("diagnosticPower", diagnosticPowerSent);
        			
            		arrIndex++;
            		
            		driveTrainDiagnosticPower++;
            		autonomousWait = 0;
        		}
        		else
        		{
        			m_NetworkTable.putNumberArray("diagnosticLeftRate", diagnosticLeftRate);
        			m_NetworkTable.putNumberArray("diagnosticRightRate", diagnosticRightRate);
        			m_NetworkTable.putNumberArray("diagnosticPower", diagnosticPowerSent);
        			
        			m_DriveTrain.arcadeDrive(0, 0);
        			driveTrainDiagnosticPower = 0;
        			autonomousWait = 0;
        			autonomousCase++;
        		}
        	}
    		break;
    	case 1:
    		if(autonomousWait >= 10)
        	{
        		if(driveTrainDiagnosticPower >= -99)
        		{
            		diagnosticPowerSent[arrIndex] = driveTrainDiagnosticPower/100;
            		diagnosticLeftRate[arrIndex] = m_DriveTrain.GetLeftSpeed();
            		diagnosticRightRate[arrIndex] = m_DriveTrain.GetRightSpeed();
            		
            		m_NetworkTable.putNumberArray("diagnosticLeftRate", diagnosticLeftRate);
        			m_NetworkTable.putNumberArray("diagnosticRightRate", diagnosticRightRate);
        			m_NetworkTable.putNumberArray("diagnosticPower", diagnosticPowerSent);
        			
            		arrIndex++;
            		
            		driveTrainDiagnosticPower--;
            		autonomousWait = 0;
        		}
        		else
        		{
        			m_NetworkTable.putNumberArray("diagnosticLeftRate", diagnosticLeftRate);
        			m_NetworkTable.putNumberArray("diagnosticRightRate", diagnosticRightRate);
        			m_NetworkTable.putNumberArray("diagnosticPower", diagnosticPowerSent);
        			
        			m_DriveTrain.arcadeDrive(0, 0);
        			driveTrainDiagnosticPower = 0;
        			autonomousWait = 0;
        			autonomousCase++;
        		}
        	}
    		break;
		default:
			break;
    	}
    	
    }
    
    public void debugRoutine(double visionTurn)
    {
    	switch(autonomousCase)
    	{
    	case 0: 
    		m_DriveTrain.ResetEncoders();
        	m_DriveTrain.ResetGyro();
    		autonomousCase++;
    		break;
    	case 1:
    		m_DriveTrain.DriveInArc(36, 1, 12, 60);
    		autonomousCase++;
    	case 2:
    		if(m_DriveTrain.isArcPIDFinished())
    		{
    			autonomousCase++;
    		}
    		break;
    	}
    }
    public void autonBoilerSide(double visionTurn)
    {
    	switch(autonomousCase)
    	{
    	case 0: // Drive out from wall to intersection
    		System.out.println("Executing Boiler Side Autonomous");
    		m_DriveTrain.ResetEncoders();
    		m_DriveTrain.ResetGyro();
    		m_DriveTrain.DriveDistance(55, 4, 24);
    		autonomousCase++;
    		break;
    	case 1: // Turn to face the gear peg
    		if(m_DriveTrain.isStraightPIDFinished())
    		{
    			m_DriveTrain.ResetEncoders();
        		m_DriveTrain.ResetGyro();
        		m_DriveTrain.TurnToAngle(60*allianceSide);
    			autonomousCase++;
    		}
    		break;
    	case 2: // Engage Peg
    		if(m_DriveTrain.isTurnPIDFinished())
    		{
    			m_DriveTrain.ResetEncoders();
    			m_DriveTrain.ResetGyro();
    			m_DriveTrain.DriveDistance(75+4, 4, 24);
    			autonomousWait = 0;
    			autonomousCase++;
    		}
    		break;
    	case 3: // Disengage Peg
    		if(m_DriveTrain.isStraightPIDFinished())
    		{
    			if(autonomousWait >= 150)
    			{
        			m_DriveTrain.ResetEncoders();
            		m_DriveTrain.ResetGyro();
            		m_DriveTrain.DriveDistance(-75, 4, 24);
            		autonomousWait = 0;
            		if(autonomousShoot)
            		{
            			autonomousCase++;
            		}
            		else
            		{
            			autonomousCase = 900;
            		}
    			}
    		}
    		break;
    	case 4: // Turn to face the boiler
    		if(m_DriveTrain.isStraightPIDFinished())
    		{
        		m_DriveTrain.ResetEncoders();
        		m_DriveTrain.ResetGyro();
        		m_DriveTrain.TurnToAngle(120*allianceSide);
        		autonomousCase++;
    		}
    		break;
    	case 5: // Transition to vision align
    		if(m_DriveTrain.isTurnPIDFinished())
    		{
    			m_DriveTrain.ResetEncoders();
    			m_DriveTrain.ResetGyro();
    			autonomousWait = 0;
    			m_DriveTrain.ArcadeDrive(0.65, 0);
    			autonomousCase++;
    		}
    		break;
    	case 6: // Drive closer to the boiler
    			m_DriveTrain.ArcadeDrive(0.65, 0);
    			if(autonomousWait >= 75)
    				autonomousCase++;
    		break;
    	case 7: // Start shooter
			m_Shooter.SetShooterSetpoint(520);
			autonomousCase++;
			autonomousWait = 0;
    		break;
    	case 8: // Shoot
			if (autonomousWait > 25)
			{
				m_Indexer.SetTalonOutput(INDEXER_SPEED);
				autonomousCase++;
			}
			break;
    	case 9:
    		break;
		default:
			if(m_DriveTrain.isStraightPIDFinished())
    		{
    			m_DriveTrain.ResetEncoders();
    			m_DriveTrain.ResetGyro();
    		}
    		break;
    	}
    }
    
    public void autonHopper(double visionTurn)
    {
    	switch(autonomousCase)
    	{
    	case 0: // Drive out from wall to hopper
    		System.out.println("Executing Hopper Autonomous");
    		m_DriveTrain.ResetEncoders();
    		m_DriveTrain.ResetGyro();
    		m_DriveTrain.DriveDistance(-121, 4, 24);
    		autonomousCase++;
    		break;
    	case 1: // Turn to trigger hopper
    		if(m_DriveTrain.isStraightPIDFinished())
    		{
    			m_DriveTrain.ResetEncoders();
    			m_DriveTrain.ResetGyro();
    			m_DriveTrain.TurnToAngle(45*allianceSide);
    			autonomousCase++;
    		}
    		break;
    	case 2: // Drive forward to collect balls
    		if(m_DriveTrain.isTurnPIDFinished())
    		{
    			autonomousCase++;
    			m_Intake.SetTalonOutput(INTAKE_SPEED);
    			autonomousWait = 0;
    				//m_DriveTrain.ResetEncoders();
        			//m_DriveTrain.ResetGyro();
        			
        			//m_DriveTrain.DriveDistance(-24, 4, 24);
    			
    		}
    		break;
    	case 3: //Drive back to boiler
    		//if(m_DriveTrain.isStraightPIDFinished())
    		//{
			
			//Wait two seconds
			if (autonomousWait > 100)
			{
    			m_DriveTrain.ResetEncoders();
        		m_DriveTrain.ResetGyro();
        		m_DriveTrain.DriveDistance(38, 4, 24);
    			m_Shooter.EnablePID();
    			m_Shooter.SetShooterSetpoint(520);
        		autonomousCase++;
			}
    		//}
    		break;
    	case 4:
    		
    		if(m_DriveTrain.isStraightPIDFinished())
    		{
    			m_DriveTrain.ResetEncoders();
    			m_DriveTrain.ResetGyro();
    			m_DriveTrain.TurnToAngle(0*allianceSide); //Aim to boiler needed?
    			autonomousCase++;
    		}
    		break;
    	case 5: //Shoot
    		if (m_DriveTrain.isTurnPIDFinished())
    		{
    			m_Indexer.SetTalonOutput(INDEXER_SPEED);
    			m_DriveTrain.arcadeDrive(0.0, 0.0);
    			m_DriveTrain.ResetEncoders();
    			m_DriveTrain.ResetGyro();
    			autonomousCase++;
    		}
			break;
    	case 6:
    		break;
		default:
			if(m_DriveTrain.isStraightPIDFinished())
    		{
    			m_DriveTrain.ResetEncoders();
    			m_DriveTrain.ResetGyro();
    		}
    		break;
    	}
    }
    
    public void autonCenter(double visionTurn)
    {
    	switch(autonomousCase)
    	{
    	case 0: // Drive out from wall and engage gear peg
    		System.out.println("Executing Center Autonomous");
    		m_DriveTrain.ResetEncoders();
    		m_DriveTrain.ResetGyro();
    		m_DriveTrain.DriveDistance(70, 4.0, 24.0);
    		autonomousCase++;
    		break;
    	case 1: // Disengage gear peg
    		if(m_DriveTrain.isStraightPIDFinished())
    		{
    			if(autonomousWait >= 100)
				{
					m_DriveTrain.ResetEncoders();
		    		m_DriveTrain.ResetGyro();
		    		m_DriveTrain.DriveDistance(-24, 4, 24);
		    		if(autonomousShoot)
            		{
            			autonomousCase++;
            		}
            		else
            		{
            			autonomousCase = 900;
            		}
    			}
    		}
    		else
    		{
    			autonomousWait = 0;
    		}
    		break;
    	case 2: // Turn to face the boiler
    		if(m_DriveTrain.isStraightPIDFinished())
    		{
    			
    			m_DriveTrain.ResetEncoders();
    			m_DriveTrain.ResetGyro();
    			
    			//TODO Blue is very off at 100deg
    			if(allianceSide == -1) // RED
    				m_DriveTrain.TurnToAngle(-98*allianceSide);
    			else // BLUE
    				m_DriveTrain.TurnToAngle(-105.5*allianceSide);
    			
    			autonomousCase++;
    		}
    		break;
    	case 3: // Transition to vision align
    		if(m_DriveTrain.isTurnPIDFinished())
    		{
    			m_DriveTrain.ResetEncoders();
        		m_DriveTrain.ResetGyro();
        		m_DriveTrain.DriveDistance(102, 8, 24);
        		
        		
    			m_Shooter.SetShooterSetpoint(525);
        		autonomousCase++;
    		}
    		break;
    	case 4: // Vision align to high boiler
    		if(m_DriveTrain.isStraightPIDFinished())
    		{
    			m_DriveTrain.arcadeDrive(0.6, 0.0);
    			autonomousWait = 0;
    			autonomousCase++;
    			
    		}
    		break;
    	case 5: // Back off the boiler for the diameter of a fuel ball
    		m_DriveTrain.DriveDistance(-5, 4, 1);
    		autonomousCase++;
    	case 6: // Shoot
    		m_DriveTrain.arcadeDrive(0.6, 0.0);
			if (autonomousWait >= 50)
			{
				m_Indexer.SetTalonOutput(INDEXER_SPEED);
				m_DriveTrain.arcadeDrive(0.0, 0.0);
				autonomousCase++;
			}
			break;
    	case 7:
    		break;
		default:
			if(m_DriveTrain.isStraightPIDFinished())
    		{
    			m_DriveTrain.ResetEncoders();
    			m_DriveTrain.ResetGyro();
    		}
    		break;
    	}
    }
    
    public void autonFeederSide(double visionTurn)
	{
		switch(autonomousCase)
    	{
    	case 0: // Drive out from wall
    		System.out.println("Executing Feeder Side Autonomous");
    		m_DriveTrain.ResetEncoders();
    		m_DriveTrain.ResetGyro();
    		//TODO modified from 60 -> 57
    		m_DriveTrain.DriveDistance(59, 4, 24);
    		autonomousCase++;
    		break;
    	case 1: // Turn to face the gear peg
    		if(m_DriveTrain.isStraightPIDFinished())
    		{
    			m_DriveTrain.ResetEncoders();
        		m_DriveTrain.ResetGyro();
        		m_DriveTrain.TurnToAngle(-60*allianceSide);
    			autonomousCase++;
    		}
    		break;
    	case 2: // Engage Peg
    		if(m_DriveTrain.isTurnPIDFinished())
    		{
    			m_DriveTrain.ResetEncoders();
    			m_DriveTrain.ResetGyro();
    			m_DriveTrain.DriveDistance(68, 4, 25);
    			autonomousWait = 0;
    			autonomousCase++;
    		}
    		break;
    	case 3: // Disengage Peg and move back to the intersection
    		if(m_DriveTrain.isStraightPIDFinished())
    		{
    			System.out.println("Wait Time: " + Integer.toString(autonomousWait));
    			if(autonomousWait >= 150)
    			{
        			m_DriveTrain.ResetEncoders();
            		m_DriveTrain.ResetGyro();
            		m_DriveTrain.DriveDistance(-65.25, 4, 25);
            		autonomousWait = 0;
            		if(autonomousShoot)
            		{
            			autonomousCase++;
            		}
            		else
            		{
            			autonomousCase = 900;
            		}
    			}
    		}
    		else
    		{
    			autonomousWait = 0;
    		}
    		break;
    	case 4: // Turn to face other end of the field
    		if(m_DriveTrain.isStraightPIDFinished())
    		{
    			m_DriveTrain.ResetEncoders();
    			m_DriveTrain.ResetGyro();
    			m_DriveTrain.TurnToAngle(57*allianceSide);
    			autonomousCase++;
    		}
    		break;
    	case 5: // Drive across the field
    		if(m_DriveTrain.isTurnPIDFinished())
    		{
    			m_DriveTrain.ResetEncoders();
    			m_DriveTrain.ResetGyro();
    			m_DriveTrain.DriveDistance(180, 8, 24);
    			autonomousCase++;
    		}
    		break;
    	default:
    		if(m_DriveTrain.isStraightPIDFinished())
    		{
    			m_DriveTrain.ResetEncoders();
    			m_DriveTrain.ResetGyro();
    		}
    		break;
    	}
    }
    
    public void arcBoilerAutonRoutine()
    {
    	switch(autonomousCase)
    	{
    	case 0: // Arc to engage the gear peg
    		m_DriveTrain.ResetEncoders();
    		m_DriveTrain.ResetGyro();
    		m_DriveTrain.DriveInArc(-104.5, 4, 24, 109.5 * allianceSide);
    		autonomousCase++;
    		break;
    	case 1: // Arc to disengage the gear peg
    		if(m_DriveTrain.isArcPIDFinished())
    		{
    			if(autonomousWait >= 75)
    			{
    				m_DriveTrain.ResetEncoders();
    				m_DriveTrain.DriveInArc(45.0, 4, 24, 113.0 * allianceSide);
    				if(autonomousShoot)
    					autonomousCase++;
    				else
    					autonomousCase = 900;
    			}
    		}
    		else
    		{
    			// Make sure autonomousWait isn't incrementing while our robot is still driving to its destination
    			autonomousWait = 0;
    		}
    		break;
    	case 2: // BLUE ONLY - Drive to boiler after engaging hopper
    		if(m_DriveTrain.isArcPIDFinished())
    		{
    			m_DriveTrain.ResetEncoders();
    			m_DriveTrain.DriveDistance(-98, 4, 24);
    			autonomousCase++;
    		}
    		break;
    	case 3: // Shoot
    		if(m_DriveTrain.isStraightPIDFinished())
    		{
    			m_DriveTrain.ResetEncoders();
    			m_Shooter.SetShooterSetpoint(252);
				m_Indexer.SetTalonOutput(INDEXER_SPEED);
				autonomousCase++;
    		}
    		else
    		{
    			autonomousWait = 0;
    		}
    		break;
    	case 4:
    		break;
		default:
			if(m_DriveTrain.isArcPIDFinished())
    		{
    			m_DriveTrain.ResetEncoders();
    			m_DriveTrain.ArcadeDrive(0, 0);
    		}
			break;
    	}
    }
    
    public void arcFeederAutonRoutine()
    {
    	switch(autonomousCase)
    	{
    	case 0: // Arc to gear peg
    		m_DriveTrain.ResetEncoders();
    		m_DriveTrain.ResetGyro();
    		m_DriveTrain.DriveInArc(0, 0, 0, 0 * allianceSide);
    		autonomousCase++;
    		break;
    	case 1: // Arc away from gear peg
    		if(m_DriveTrain.isArcPIDFinished())
    		{
    			if(autonomousWait >= 75)
    			{
    				m_DriveTrain.ResetEncoders();
    				m_DriveTrain.DriveInArc(-0, 0, 0, -0 * allianceSide);
    				if(autonomousShoot)
    					autonomousCase++;
    				else
    					autonomousCase = 900;
    			}
    		}
    		else
    		{
    			// Make sure autonomousWait isn't incrementing while our robot is still driving to its destination
    			autonomousWait = 0;
    		}
    		break;
    	case 2: // Drive down the field to the edge of the neutral zone
    		if(m_DriveTrain.isArcPIDFinished())
    		{
    			m_DriveTrain.ResetEncoders();
    			m_DriveTrain.DriveDistance(180, 8, 24);
    			autonomousCase++;
    		}
    		break;
    	case 3:
    		if(m_DriveTrain.isStraightPIDFinished())
    		{
    			m_DriveTrain.ResetEncoders();
    			m_DriveTrain.ArcadeDrive(0, 0);
    		}
    		break;
    	default:
    		if(m_DriveTrain.isArcPIDFinished())
    		{
    			m_DriveTrain.ResetEncoders();
    			m_DriveTrain.ArcadeDrive(0, 0);
    		}
    		break;
    	}
    }
    
    public void arcHopperAutonRoutine()
    {
    	
    }
    
    public void dumbAutonRoutine()
    {
    	switch(autonomousCase)
    	{
    	case 0:
    		m_DriveTrain.ArcadeDrive(0.6, 0);
    		autonomousWait++;
    		if(autonomousWait >= 75)
    		{
    			autonomousWait = 0;
    			autonomousCase++;
    		}
    		break;
    	case 1:
    		if(autonomousWait >= 100)
    		{
        		autonomousWait = 0;
    			autonomousCase++;
    		}
    		break;
    	case 2:
    		m_DriveTrain.ArcadeDrive(-0.6, 0.0);
    		if(autonomousWait >= 25)
    		{
    			m_DriveTrain.ArcadeDrive(0.0, 0.0);
    			autonomousWait = 0;
    			autonomousCase++;
    		}
    		break;
    	case 3:
    		if(allianceSide == -1) // RED
				m_DriveTrain.TurnToAngle(-100*allianceSide);
			else // BLUE
				m_DriveTrain.TurnToAngle(-102.5*allianceSide);
    		break;
    	case 4:
    		if(m_DriveTrain.isTurnPIDFinished())
    		{
    			autonomousCase++;
    			autonomousWait = 0;
    		}
    		break;
    	case 5:
    		m_DriveTrain.ArcadeDrive(0.6, 0);
    		
    		if(autonomousWait >= 150)
    		{
    			m_DriveTrain.ArcadeDrive(0.0, 0.0);
    			autonomousWait = 0;
    			autonomousCase++;
    		}
    	case 6:
    		if(autonomousWait == 0)
    		{
        		m_Shooter.EnablePID();
    			m_Shooter.SetShooterSetpoint(530);
    		}
    		
    		if(autonomousWait >= 50)
    		{
    			m_Indexer.SetTalonOutput(INDEXER_SPEED);
    		}
    		break;
    	}
    }
    
    
    
    
    public void teleopPeriodic()
    {
    	/**
         * This function is called periodically during operator control
         */
    	
    	if(autonomousCase != 0)
    	{
    		autonomousCase = 0;
    		m_DriveTrain.DisablePIDControl();
    	}
    	
    	GetDashboardData();
    	WriteDashboardData();
    	
    	/*double centerX;
    	synchronized (m_ImgLock) {
    		centerX = this.centerX;
    	}
    	double turn = (centerX - (IMG_WIDTH / 2))/(IMG_WIDTH/2) * (CAMERA_ANGLE/2);
    	if (centerX == 0)
    	{
    		turn = 0;
    	}*/
    	//Drivetrain methods
    	/*if(m_RobotInterface.GetDriverLeftBumper())
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
    	{*/
        	
    	//}
    	arcadeDrive();
    	m_DriveTrain.WriteDashboardData();
    	
    	//Shooter methods
    	runShooter();
    	
    	//Other
    	runIndexer();
    	runIntake();
    	runScaler();
    	
    	
    	
    	//Misc variable updates
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
    	if(m_RobotInterface.GetDriverLeftTrigger())
    	{
    	 	m_DriveTrain.ArcadeDrive(m_RobotInterface.GetDriverLeftY()*0.7, m_RobotInterface.GetDriverRightX()*0.7);
    	}
    	else
    	{
    	 	m_DriveTrain.ArcadeDrive(m_RobotInterface.GetDriverLeftY(), m_RobotInterface.GetDriverRightX());
    	}
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
    	double CLOSE_SPEED = .65;
    	double MEDIUM_SPEED = .70;
    	double FAR_SPEED = .75;
    	GetDashboardData();
    	
    	boolean useRPM = m_RobotInterface.GetOperatorButton(12);
    	
    	if (m_RobotInterface.GetOperatorButton(11))
     	{
     		// BEST FIT BTN 11
     		m_Shooter.SetShooterSetpoint(shooterRPMBF);
     	}
    	else if(!useRPM)
    	{
        	if (m_RobotInterface.GetOperatorButton(6))
        	{
        		shooterRPM = .68;
        		m_Shooter.SetTalonOutput(shooterRPM);
        	} 
        	else if (m_RobotInterface.GetOperatorButton(5))
        	{
        		shooterRPM = CLOSE_SPEED;
        		m_Shooter.SetTalonOutput(shooterRPM);
        	} 
        	else if (m_RobotInterface.GetOperatorButton(3)) 
        	{
        		shooterRPM = MEDIUM_SPEED;
        		m_Shooter.SetTalonOutput(shooterRPM);
        	} 
        	else if (m_RobotInterface.GetOperatorButton(4)) 
        	{
        		shooterRPM = FAR_SPEED;
        		m_Shooter.SetTalonOutput(shooterRPM);
        	}
        	else if (m_RobotInterface.GetOperatorButton(7)) 
        	{
        		shooterRPM = .8;
        		m_Shooter.SetTalonOutput(shooterRPM);
        	}
        	else if (m_RobotInterface.GetOperatorButton(8)) 
        	{
        		shooterRPM = .85;
        		m_Shooter.SetTalonOutput(shooterRPM);
        	} 
        	else if (m_RobotInterface.GetOperatorButton(9)) 
        	{
        		shooterRPM = .9;
        		m_Shooter.SetTalonOutput(shooterRPM);
        	}
        	else if (m_RobotInterface.GetOperatorButton(10)) 
        	{
        		shooterRPM = .95;
        		m_Shooter.SetTalonOutput(shooterRPM);
        	}
        	else
        	{
        		shooterRPM = 0;
        		m_Shooter.SetTalonOutput(shooterRPM);
        	}
    	}
    	else if(useRPM)
    	{
    		if (m_RobotInterface.GetOperatorButton(5))
        	{
        		shooterRPM = 520;
        		m_Shooter.SetShooterSetpoint(shooterRPM);
        	} 
        	else if (m_RobotInterface.GetOperatorButton(6))
        	{
        		shooterRPM = 530;
        		m_Shooter.SetShooterSetpoint(shooterRPM);
        	} 
        	else if (m_RobotInterface.GetOperatorButton(3)) 
        	{
        		shooterRPM = 540;
        		m_Shooter.SetShooterSetpoint(shooterRPM);
        	} 
        	else if (m_RobotInterface.GetOperatorButton(4)) 
        	{
        		shooterRPM = 550;
        		m_Shooter.SetShooterSetpoint(shooterRPM);
        	}
        	else
        	{
        		m_Shooter.DisablePID();
        	}
    	}
    	else if (m_RobotInterface.GetDriverY())
    	{
    		m_Shooter.SetShooterSetpoint(shooterRPMBF);
    	}
    	else
    	{
    		m_Shooter.DisablePID();
    		m_Shooter.SetTalonOutput(0);
    	}
    	//if(m_RobotInterface.GetDriverRightBumper())
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
    	/*else if(m_RobotInterface.GetDriverRightTrigger())
    	{
    		System.out.println("Shooter entered");
    		//Enable the PID controller for the shooter using a rate calculated read from the dashboard
    		//m_Shooter.SetShooterSetpoint(shooterRPM);
    		//m_Shooter.EnablePID();
    		m_Shooter.SetTalonOutput(shooterRPM);
			SmartDashboard.putBoolean("Shooter on target", m_Shooter.ShooterOnTarget());
			
    		/*if(m_Shooter.ShooterOnTarget())
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
    		m_Shooter.SetShooterSetpoint(0);
    		m_Shooter.DisablePID();

    		m_LightSystem.setDefault();
    	}*/
    }
    
    public double calculateShooterRPM()
    {
    	//y = 11.815x2 + 140.58x + 1348.4 
		double distanceFt = m_Lidar.getDistanceFt();
		double shooterRate = (11.815*(distanceFt*distanceFt) + 140.58*distanceFt+1348.4);
		return shooterRate;
    }
    
    //Indexer methods
    public void runIndexer() 
    {
    	if(m_RobotInterface.GetDriverRightBumper())
 		{
    		m_Indexer.SetTalonOutput(INDEXER_SPEED);
 		}
 	    else
 	    {
 	    	m_Indexer.SetTalonOutput(0);
 	    }
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
    
    //Scaler methods
    public void runScaler() 
    {
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
    	shooterRPMBF = SmartDashboard.getNumber("shooterRPMBF", DEFAULT_SHOOTER_RATE);
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