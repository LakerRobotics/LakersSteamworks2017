package org.usfirst.frc.team5053.robot;

import org.usfirst.frc.team5053.robot.RobotInterfaceMap.JoystickType;
import org.usfirst.frc.team5053.robot.Sensors.LidarLite;
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
import edu.wpi.first.wpilibj.IterativeRobot;
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
	private final double INDEXER_SPEED = .7;
	private final double SCALER_SPEED = -.85;

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
	private boolean autonShoot;
	
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
    	m_Shooter = new Shooter(m_RobotControllers.getShooter(), m_RobotSensors.getShooterEncoder());
    	m_Intake = new Intake(m_RobotControllers.getIntake());
    	m_Indexer = new Indexer(m_RobotControllers.getIndexer());
    	m_Scaler = new Scaler(m_RobotControllers.getScaler());
    	
    	//Misc Subsystem Initialization
    	m_LightSystem = new LightSystem(m_RobotSensors.getRed(), m_RobotSensors.getBlue(), m_RobotSensors.getGreen(), DriverStation.getInstance().getAlliance());
    	m_LightSystem.setDefault();
    	m_Lidar = m_RobotSensors.getLidar();
    	m_Lidar.start();
    	
    	shooterRPM = .65;
    	
    	//Camera Initialization
    	m_Camera = CameraServer.getInstance().startAutomaticCapture();
    	m_Camera.setExposureManual(1);
    	m_Camera.setFPS(30);
    	m_Camera.setBrightness(1);
        m_Camera.setResolution(IMG_WIDTH, IMG_HEIGHT);
    	
        centerX = (IMG_WIDTH/2);
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
    	autonomousWait = 0;
    	autonShoot = SmartDashboard.getBoolean("shoot");
    	
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
    }

    public void autonomousPeriodic()
    {
    	/**
         * This function is called periodically during autonomous
         */
    	
    	double centerX;
    	//Updates our vision angle
    	synchronized (m_ImgLock) {
    		centerX = this.centerX;
    	}
    	double turn = (centerX - (IMG_WIDTH / 2))/(IMG_WIDTH/2) * (CAMERA_ANGLE/2);

    	SmartDashboard.putNumber("Vision Turn", turn);
    	
    	switch((int) SmartDashboard.getNumber("autonRoutine", 0))
    	{
    	case 0: //NO AUTON
    		break;
    	case 1: //CENTER
    		autonCenter(turn);
    		break;
    	case 2: //RIGHT
    		autonFeederSide(turn);
    		break;
    	case 3: //LEFT
    		autonBoilerSide(turn);
    		break;
    	case 4:
    		debugTurn(turn);
    		break;
		default: //NO AUTON
			break;
    	}
    	
		m_DriveTrain.WriteDashboardData();
		m_Shooter.WriteDashboardData();
    }
    public void debugTurn(double visionTurn)
    {
    	switch(autonomousCase)
    	{
    	case 0: 
    		m_DriveTrain.ResetEncoders();
        	m_DriveTrain.ResetGyro();
    		autonomousCase++;
    		break;
    	case 1:
    		autonomousCase++;
    		System.out.println("Seeking Target");
    		visionAlign(visionTurn /*Vision turn aligns to the reflective tape on the high goal*/);
    		break;
    	
    	case 2:
    		if(!(Math.abs(previousVisionTurn) == visionTurn))
    		{
    			previousVisionTurn = visionTurn;
        		visionAlign(visionTurn /*Vision turn aligns to the reflective tape on the high goal*/);
        		System.out.println(Double.toString(visionTurn));
    		}
    		if(!isVisionTurnRunning)
    		{
    			autonomousCase++;
    		}
    		break;
    	case 3:
    		System.out.println("On Target");
    		autonomousCase++;
    		break;
    	case 4:
    		break;
    	}
    }
    public void autonBoilerSide(double visionTurn)
    {
    	switch(autonomousCase)
    	{
    	case 0: //Drive out from wall
    		System.out.println("Executing Boiler Side Autonomous");
    		m_DriveTrain.ResetEncoders();
    		m_DriveTrain.ResetGyro();
    		m_DriveTrain.DriveDistance(48 /*Distance to peg-start intersection*/, 4, 24);
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
    			m_DriveTrain.DriveDistance(70/*distance to the gear peg*/, 4, 24);
    			autonomousCase++;
    		}
    		break;
    	case 3: //Disengage Peg
    		if(m_DriveTrain.isStraightPIDFinished())
    		{
    			autonomousWait++;
    			System.out.println("Wait Time: " + Integer.toString(autonomousWait));
    			if(autonomousWait >= 150)
    			{
        			m_DriveTrain.ResetEncoders();
            		m_DriveTrain.ResetGyro();
            		m_DriveTrain.DriveDistance(-75/*distance back to the intersection*/, 4, 24);
            		autonomousWait = 0;
            		if(autonShoot)
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
    	case 4: //Turn to face the boiler
    		if(m_DriveTrain.isStraightPIDFinished())
    		{
        		m_DriveTrain.ResetEncoders();
        		m_DriveTrain.ResetGyro();
        		m_DriveTrain.TurnToAngle(120*allianceSide/*Turns the robot around so the high goal is within view for vision targeting*/);
        		autonomousCase++;
    		}
    		break;
    	case 5: //Transition to vision align
    		if(m_DriveTrain.isTurnPIDFinished())
    		{
    			m_DriveTrain.ResetEncoders();
    			m_DriveTrain.ResetGyro();
    			autonomousCase++;
    		}
    		break;
    	case 6: //Transition to vision align
    		
        		m_DriveTrain.DriveDistance(24 /*Distance to engage the gear peg*/, 4, 24);
        		autonomousCase++;
    		break;
    	case 7: //Vision align to high boiler
    		if(m_DriveTrain.isStraightPIDFinished())
    		{
    			m_Shooter.SetTalonOutput(.68);
    			autonomousCase++;
    			autonomousWait = 0;
    		}
    		break;
    	case 8: //Shoot
			autonomousWait++;
			if (autonomousWait > 50)
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
    
    public void autonCenter(double visionTurn)
    {
    	switch(autonomousCase)
    	{
    	case 0: //Drive out from wall and engage gear peg
    		System.out.println("Executing Center Autonomous");
    		m_DriveTrain.ResetEncoders();
    		m_DriveTrain.ResetGyro();
    		m_DriveTrain.DriveDistance(70 /*Distance to engage the gear peg*/, 4, 24);
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
		    		m_DriveTrain.DriveDistance(-24/*Distance to disengage gear peg*/, 4, 24);
		    		if(autonShoot)
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
    	case 2: // Turn to face the boiler
    		if(m_DriveTrain.isStraightPIDFinished())
    		{
    			m_DriveTrain.ResetEncoders();
    			m_DriveTrain.ResetGyro();
    			m_DriveTrain.TurnToAngle(-100*allianceSide/*Angle to face the boiler*/);
    			autonomousCase++;
    		}
    		break;
    	case 3: //Transition to vision align
    		if(m_DriveTrain.isTurnPIDFinished())
    		{
    			m_DriveTrain.ResetEncoders();
        		m_DriveTrain.ResetGyro();
        		m_DriveTrain.DriveDistance(84 /*Distance to engage the gear peg*/, 8, 24);
        		autonomousCase++;
    		}
    		break;
    	case 4: //Vision align to high boiler
    		if(m_DriveTrain.isStraightPIDFinished())
    		{
    			m_Shooter.SetTalonOutput(.70);
    			m_DriveTrain.arcadeDrive(.6, 0.0);
    			autonomousCase++;
    			autonomousWait = 0;
    		}
    		break;
    	case 5: //Shoot
			autonomousWait++;
			if (autonomousWait > 150)
			{
				m_Indexer.SetTalonOutput(INDEXER_SPEED);
				m_DriveTrain.arcadeDrive(0.0, 0.0);
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
    
    public void autonFeederSide(double visionTurn)
	{
		switch(autonomousCase)
    	{
    	case 0: //Drive out from wall
    		System.out.println("Executing Feeder Side Autonomous");
    		m_DriveTrain.ResetEncoders();
    		m_DriveTrain.ResetGyro();
    		m_DriveTrain.DriveDistance(63.434 /*Distance to peg-start intersection*/, 2, 24);
    		autonomousCase++;
    		break;
    	case 1: //Turn to face the gear peg
    		if(m_DriveTrain.isStraightPIDFinished())
    		{
    			m_DriveTrain.ResetEncoders();
        		m_DriveTrain.ResetGyro();
        		m_DriveTrain.TurnToAngle(-60*allianceSide/*Angle to face the gear peg*/);
    			autonomousCase++;
    		}
    		break;
    	case 2: //Engage Peg
    		if(m_DriveTrain.isTurnPIDFinished())
    		{
    			m_DriveTrain.ResetEncoders();
    			m_DriveTrain.ResetGyro();
    			m_DriveTrain.DriveDistance(60/*Distance to gear peg*/, 4, 24);
    			autonomousCase++;
    		}
    		break;
    	case 3: //Disengage Peg and move back to the intersection
    		if(m_DriveTrain.isStraightPIDFinished())
    		{
    			autonomousWait++;
    			System.out.println("Wait Time: " + Integer.toString(autonomousWait));
    			if(autonomousWait >= 150)
    			{
        			m_DriveTrain.ResetEncoders();
            		m_DriveTrain.ResetGyro();
            		m_DriveTrain.DriveDistance(-115.25/*Distance to disengage the peg*/, 4, 24);
            		autonomousWait = 0;
            		if(autonShoot)
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
    	case 4: //Turn to face the boiler
    		if(m_DriveTrain.isStraightPIDFinished())
    		{
    			m_DriveTrain.ResetEncoders();
    			m_DriveTrain.ResetGyro();
    			m_DriveTrain.TurnToAngle(138.352*allianceSide/*Turn to have back face the driver station wall*/);
    			autonomousCase++;
    		}
    		break;
    	case 5: //Drive to the boiler
    		if(m_DriveTrain.isTurnPIDFinished())
    		{
    			m_DriveTrain.ResetEncoders();
    			m_DriveTrain.ResetGyro();
    			m_DriveTrain.DriveDistance(-228.692/*Distance to the boiler*/, 4, 24);
    			autonomousCase++;
    		}
    		break;
    	case 7: //Vision align to high boiler
    		if(m_DriveTrain.isStraightPIDFinished())
    		{
    			m_Shooter.SetTalonOutput(.68);
    			autonomousCase++;
    			autonomousWait = 0;
    		}
    		break;
    	case 8: //Shoot
    		autonomousWait++;
    		if (autonomousWait > 50)
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
    	
    	double centerX;
    	synchronized (m_ImgLock) {
    		centerX = this.centerX;
    	}
    	double turn = (centerX - (IMG_WIDTH / 2))/(IMG_WIDTH/2) * (CAMERA_ANGLE/2);
    	if (centerX == 0)
    	{
    		turn = 0;
    	}
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
    	if(m_RobotInterface.GetDriverLeftTrigger() > 0)
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
    	
    	
    	if (m_RobotInterface.GetOperatorButton(3))
    	{
    		shooterRPM = .68;
    		m_Shooter.SetTalonOutput(shooterRPM);
    	} else if (m_RobotInterface.GetOperatorButton(4)){
    		shooterRPM = CLOSE_SPEED;
    		m_Shooter.SetTalonOutput(shooterRPM);
    	} else if (m_RobotInterface.GetOperatorButton(5)) {
    		shooterRPM = MEDIUM_SPEED;
    		m_Shooter.SetTalonOutput(shooterRPM);
    	} else if (m_RobotInterface.GetOperatorButton(6)) {
    		shooterRPM = FAR_SPEED;
    		m_Shooter.SetTalonOutput(shooterRPM);
    	} else if (m_RobotInterface.GetOperatorButton(7)) {
    		shooterRPM = .8;
    		m_Shooter.SetTalonOutput(shooterRPM);
    	} else if (m_RobotInterface.GetOperatorButton(8)) {
    		shooterRPM = .85;
    		m_Shooter.SetTalonOutput(shooterRPM);
    	} else if (m_RobotInterface.GetOperatorButton(9)) {
    		shooterRPM = .9;
    		m_Shooter.SetTalonOutput(shooterRPM);
    	} else if (m_RobotInterface.GetOperatorButton(10)) {
    		shooterRPM = .95;
    		m_Shooter.SetTalonOutput(shooterRPM);
    	} else if (m_RobotInterface.GetOperatorButton(11)) {
    		shooterRPM = 1;
    		m_Shooter.SetTalonOutput(shooterRPM);
    	} else {
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
    	/*else if(m_RobotInterface.GetDriverRightTrigger() > 0)
    	{
    		System.out.println("Shooter entered");
    		//Enable the PID controller for the shooter using a rate calculated read from the dashboard
    		//m_Shooter.SetShooterSetpoint(shooterRPM);
    		//m_Shooter.EnablePID();
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
    	//shooterRPM = SmartDashboard.getNumber("shooterRPM", DEFAULT_SHOOTER_RATE);
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