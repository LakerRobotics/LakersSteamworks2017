package org.usfirst.frc.team5053.robot;

import org.usfirst.frc.team5053.robot.RobotInterfaceMap.joystickType;
import org.usfirst.frc.team5053.robot.Subsystems.GearManipulator;
import org.usfirst.frc.team5053.robot.Subsystems.DriveTrain;
import org.usfirst.frc.team5053.robot.Subsystems.Intake;
import org.usfirst.frc.team5053.robot.Subsystems.Shooter;
import org.usfirst.frc.team5053.robot.Subsystems.Utilities.AnglePIDWrapper;
import org.usfirst.frc.team5053.robot.Subsystems.Utilities.DistancePIDWrapper;
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
	public static RobotSensorMap m_RobotSensors;
	
	
	
	//Robot Subsystem Declaration
	public static DriveTrain m_DriveTrain;
	GearManipulator m_Arm;
	Intake m_Intake;
	Shooter m_Shooter;
	
	AnglePIDWrapper m_AnglePIDWrapper;
	DistancePIDWrapper m_DistancePIDWrapper;
	
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

	private final int IMG_WIDTH		= 320;
	private final int IMG_HEIGHT 	= 240;
	private final int CAMERA_ANGLE 	= 52;
	
	private VisionThread m_VisionThread;
	private double m_CenterX = 0.0;
	private double m_VisionTurn;
	
	private final Object m_ImgLock = new Object();
	
	private boolean isDriveTrainPIDFinished = false;
	private boolean m_HasTarget;
	private int autonomousCase;
	
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
    	m_DriveTrain = new DriveTrain(m_RobotControllers.getLeftDrive(), m_RobotControllers.getRightDrive(), m_RobotSensors.getLeftDriveEncoder(), m_RobotSensors.getRightDriveEncoder(), m_RobotSensors.getGyro());
    	m_Arm = new GearManipulator(m_RobotControllers.getGearManipulator(), m_RobotSensors.getGearManipulatorEncoder());
    	
    	m_Shooter = new Shooter(m_RobotControllers.getShooter(), m_RobotSensors.getShooterEncoder());
    	m_Intake = new Intake(m_RobotControllers.getIntake());
    	
    	m_AnglePIDWrapper = new AnglePIDWrapper(m_DriveTrain);
    	m_DistancePIDWrapper = new DistancePIDWrapper(m_DriveTrain);
    	
    	autonomousCase = 0;
    	
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
        
        System.out.println(m_DriveTrain.GetAverageDistance());
        System.out.println(m_DriveTrain.GetPIDSetpoint());
        
        m_HasTarget = false;
        
    	
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
    	this.m_VisionTurn = (centerX - (IMG_WIDTH / 2))/(IMG_WIDTH/2) * (CAMERA_ANGLE/2);

    	SmartDashboard.putNumber("Vision Turn", m_VisionTurn);
    	//m_DriveTrain.arcadeDrive(-0.6, turn * 0.005);
    	
        /**
         * This function is called periodically during autonomous
         */
    	switch(autonomousCase)
    	{
    	case 0:
    		m_DriveTrain.ResetEncoders();
    		m_DriveTrain.ResetAngle();
    		m_DriveTrain.SetPIDSetpoint(2000, 0);
    		m_DriveTrain.EnablePID();
    		m_DriveTrain.WriteDashboardData();
    		autonomousCase++;
    		System.out.println("Incrementing Case");
    		break;
    	case 1:
    		m_DriveTrain.WriteDashboardData();
    		if(m_DriveTrain.DistanceOnTarget())
    		{
    			m_DriveTrain.DisablePID();
    		}
    		break;
    	}
    }


    public void teleopPeriodic()
    {
    	SmartDashboard.putNumber("Vision CenterX", m_CenterX);
    	GetDashboardData();
    	m_DriveTrain.WriteDashboardData();
    	
    	double centerX;
    	synchronized (m_ImgLock) {
    		centerX = this.m_CenterX;
    	}
    	
    	
    	    	
    	if(m_RobotInterface.GetDriverB() && !m_HasTarget) {
    		
    		m_DriveTrain.ResetAngle();
    		m_DriveTrain.ResetEncoders();
    		this.m_VisionTurn = (centerX - (IMG_WIDTH / 2))/(IMG_WIDTH/2) * (CAMERA_ANGLE/2);
    		m_DriveTrain.SetPIDSetpoint(0, m_VisionTurn);
    		m_DriveTrain.EnablePID();
    		m_HasTarget = true;
    	}
    	else if(!m_RobotInterface.GetDriverB())
    	{
    		m_DriveTrain.DisablePID();
    		arcadeDrive();
    		m_HasTarget = false;
    	}
    	
        /**
         * This function is called periodically during operator control
         */
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
    	m_DriveTrain.TeleopDrive(-m_RobotInterface.GetDriverLeftY(), m_RobotInterface.GetDriverRightX());
    }
    
    //We don't have an arm... yet!
    public void manualArmControl() 
    {
    	//Gives the operator manual control over the arm if setpoints aren't working (sensor failure)
    	m_Arm.DisablePID();
        m_Arm.SetTalonOutput(-m_RobotInterface.GetOperatorJoystick().getRawAxis(1));
    }
    public void getTargetOffset()
    {
    	double centerX;
    	synchronized (m_ImgLock) {
    		centerX = this.m_CenterX;
    	}
    	this.m_VisionTurn = (centerX - (IMG_WIDTH / 2))/(IMG_WIDTH/2) * (CAMERA_ANGLE/2);
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