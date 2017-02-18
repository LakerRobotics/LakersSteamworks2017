package org.usfirst.frc.team5053.robot;

import org.usfirst.frc.team5053.robot.RobotInterfaceMap.JoystickType;
import org.usfirst.frc.team5053.robot.Subsystems.DriveTrainMotionControl;
import org.usfirst.frc.team5053.robot.Subsystems.Indexer;
import org.usfirst.frc.team5053.robot.Subsystems.Intake;
import org.usfirst.frc.team5053.robot.Subsystems.LightSystem;
//MixerDoesntExist import org.usfirst.frc.team5053.robot.Subsystems.Mixer;
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
	//MixerDoesntExist	private Mixer m_Mixer;
	private LightSystem m_LightSystem;
	
	//Vision declaration
	private UsbCamera m_Camera;
	private VisionThread m_VisionThread;
	private boolean visionHasTarget = false;
	private boolean visionOnThreadHasTarget = false;
	private int blinkCounterForHaveTarget = 0;
	private double centerX;
    double visionAngleMissAligned;
    double visionOnThreadAngleMissAligned;
	private double visionDistanceToBoilerTarget;  // in Feet
	private double visionOnThreadDistanceToBoilerTarget;  // in Feet
	
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
	private final int CAMERA_ANGLE 	= 52; //?62 see comments in notes 2lines down
	private final int CAMERA_VERTICAL_VIEW_ANGLE = 37;
	// Chief Delphi 2/20/2016 says Horizontal 61 Degrees Vertical 34.3 Degrees
	// https://www.chiefdelphi.com/forums/showthread.php?p=1543606 
	// But the references used by the post were updated 4/23/2016
	// https://dl2jx7zfbtwvr.cloudfront.net/specsheets/WEBC1010.pdf says diagonal Field of view is 68.5
	// Using the following reference interpolate between 60 and 70 and get 61.39 Horizontal and 36.955
	// http://vrguy.blogspot.com/2013/04/converting-diagonal-field-of-view-and.html
	
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
//    	m_RobotInterface = new RobotInterfaceMap(JoystickType.XBOX, JoystickType.JOYSTICK);
    	m_RobotInterface = new RobotInterfaceMap(JoystickType.JOYSTICK, JoystickType.JOYSTICK);

    	m_RobotControllers = new RobotControllerMap();
    	m_RobotSensors = new RobotSensorMap();    	
    	m_RobotSensors.getLidar().start(); // start taking distance reading
    	
    	//Robot Subsystem Initialization
    	m_DriveTrain = new DriveTrainMotionControl(m_RobotControllers.getLeftDrive(), m_RobotControllers.getRightDrive(), m_RobotSensors.getLeftDriveEncoder(), m_RobotSensors.getRightDriveEncoder(), m_RobotSensors.getGyro());
    	m_Shooter = new Shooter(m_RobotControllers.getShooter(), m_RobotSensors.getShooterEncoder());
    	m_Intake  = new Intake( m_RobotControllers.getIntake());
    	m_Indexer = new Indexer(m_RobotControllers.getIndexer());
    	//MixerDoesntExist    	m_Mixer = new Mixer(m_RobotControllers.getMixer());
    	m_Scaler  = new Scaler( m_RobotControllers.getScaler());
    	
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
    	visionOnThreadDistanceToBoilerTarget  = 4.0; //in Feet
       

          m_VisionThread = new VisionThread(m_Camera, new GRIPVision(), pipeline -> {
    	 
    	        	SmartDashboard.putBoolean("Is empty", pipeline.filterContoursOutput().isEmpty());
    	            if (!pipeline.filterContoursOutput().isEmpty()) {
    	                Rect r = Imgproc.boundingRect(pipeline.filterContoursOutput().get(0));
    	                synchronized (m_ImgLock) {
    	                	visionOnThreadHasTarget= true;
    	                	centerX = r.x + (r.width / 2);
    	                	visionOnThreadAngleMissAligned = (centerX - (IMG_WIDTH / 2))/(IMG_WIDTH/2) * (CAMERA_ANGLE/2);

    	                    
    	                    // Calcualte distance from target
    	                    // 2016 Game:
    	                    // http://wpilib.screenstepslive.com/s/4485/m/24194/l/288985-identifying-and-processing-the-targets
    	                    
    	                    // tanget(angle) = x/y or in this case x is width and y is direct distance to the target
    	                    // tangent(target_height_angle) = target_height_in/direct_distance --> solve for 
    	                    // Note as we get closer the apparent height of the target will 
    	                    // SKIP THIS found they already figured it out, although they warn about inaccuracies, we should switch to lidar
    	                    // 2017 Game:
    	                    // http://wpilib.screenstepslive.com/s/4485/m/24194/l/683625-processing-images-from-the-2017-frc-game
    	                    //   distance = Target height in ft. (10/12) * YRes / (2 * PixelHeight * tan(viewAngle of camera))
    	                    boolean GET_DISTANCE_FROM_VERTICALLY=false;
    	                    if(GET_DISTANCE_FROM_VERTICALLY){
    	                       double target_height_pixles = r.height;
    	                       double target_height_in = 4; //10 inches if we have both
    	                       this.visionOnThreadDistanceToBoilerTarget = (target_height_in/12)  /* converted to feet -- vision height of the rectangle assumes only have the top one*/
    	                    		                  * IMG_HEIGHT  / (2*target_height_pixles*Math.tan((2*Math.PI/360)*CAMERA_VERTICAL_VIEW_ANGLE));
    	                    }
    	                    else{
    	                        double target_width_pixles = r.width;
    	                        double target_width_in = 15; //
    	                        this.visionOnThreadDistanceToBoilerTarget = (target_width_in/12)  /* converted to feet -- vision width of the rectangle*/
    	                     		                  * IMG_WIDTH / (2*target_width_pixles*Math.tan((2*Math.PI/360)*CAMERA_ANGLE));
    	                    }
    	                    SmartDashboard.putNumber("Vision CenterX", centerX);
    	                    SmartDashboard.putNumber("VisionAngleMissAligned", visionAngleMissAligned);
    	                    SmartDashboard.putNumber("VisionDistanceToBoilerTarget", visionDistanceToBoilerTarget);
    	                    SmartDashboard.putNumber("VisionTargetVerticalHeightInt", r.height);
    	                    SmartDashboard.putNumber("VisionTargetWidthInt", r.width);
    	                }
    	            }
    	           else{
    	            	visionOnThreadHasTarget= false;
    	//                SmartDashboard.putNumber("Vision CenterX", centerX); // Leave this because maybe we just got to close to still see the target, so use last angle
    	//                SmartDashboard.putNumber("VisionDistanceToBoilerTarget", 0); // Zero out? 
    	            }
    	            SmartDashboard.putBoolean("VisionHaveTarget", visionOnThreadHasTarget);
    	        });
    	       
    	        m_VisionThread.start();
    }

    public void autonomousInit() 
    {
    	 /**
         * This function is called once when autonomous begins
         */
    	GetDashboardData();
    }
    
    private enum AutonRoutines {TEST_SQUARE, TEST_ARCH, NONE};
    /**
     * called repeatedly during Autonomous
     */
    public void autonomousPeriodic()
    {
    	double distanceToBoilerTarget;
    	double angleMissAligned;
    	synchronized (m_ImgLock) {
    		angleMissAligned = this.visionAngleMissAligned;
        	visionDistanceToBoilerTarget = this.visionDistanceToBoilerTarget;
    	}

    	SmartDashboard.putNumber("Vision Turn", angleMissAligned);
    	
    	AutonRoutines autonToRun = AutonRoutines.TEST_SQUARE; 
    	switch(autonToRun){
    	case TEST_SQUARE:
        	autonDriveSquare();
    	break;
    	case TEST_ARCH:
        	autonDriveSquare();
    	break;
    	case NONE:
        	
    	break;
    	default:
    		
    		break;
    	}
    }

	/**
	 * 
	 */
	protected void autonDriveSquare() {
		double sizeOfSquare = 5;//inches
		double maxSpeed = 3;// ft/sec
		switch(autonomousCase)
    	{
    	case 0:
//    		m_DriveTrain.ResetEncoders();
//    		m_DriveTrain.ResetGyro(); //Don't reset, we want it to keep its field oreintation

    		// will return true when the drive distance is coplete
    		if(m_DriveTrain.DriveDistance(sizeOfSquare, maxSpeed, 7))// Distance in inches, maxspeed in ft/sec, rampupdown,
    		{    			
        		m_DriveTrain.WriteDashboardData();
        		autonomousCase++;
        		System.out.println("Incrementing Case");
    		}; 
    		break;
    	case 1:
    		if(m_DriveTrain.TurnToAngle(90)){autonomousCase++;}
    		break;
    	case 2:
    		if(m_DriveTrain.DriveDistance(sizeOfSquare, maxSpeed, 7))// Distance in inches, maxspeed in ft/sec, rampupdown,
    		{autonomousCase++;}; 
    		break;
    	case 3:
    		if(m_DriveTrain.TurnToAngle(90)){autonomousCase++;}
    		break;
       	case 4:
    		if(m_DriveTrain.DriveDistance(sizeOfSquare, maxSpeed, 7))// Distance in inches, maxspeed in ft/sec, rampupdown,
    		{autonomousCase++;}; 
    		break;
     	case 5:
     		if(m_DriveTrain.TurnToAngle(90)){autonomousCase++;}
    		break;
    	case 6:
    		if(m_DriveTrain.DriveDistance(sizeOfSquare, maxSpeed, 7))// Distance in inches, maxspeed in ft/sec, rampupdown,
    		{autonomousCase++;}; 
    		break;
    	case 7:
    		if(m_DriveTrain.TurnToAngle(90)){autonomousCase++;}
    		break;
    	default:
    		break;
    	}
	}

	/**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic()
    {    	
    	synchronized (m_ImgLock) {
    		visionHasTarget = this.visionOnThreadHasTarget;
    		visionAngleMissAligned = this.visionOnThreadAngleMissAligned;
        	visionDistanceToBoilerTarget = this.visionOnThreadDistanceToBoilerTarget;
    	}
    	this.manageLightsForVisionTargetAcquired();
    	
    	manageShootingAndDriving();
    	
    	manageIntake();

    	runScaler();	
    	
//Instead use light to indicate info about targeting teleopLightLoops++;
    }

	/**
	 * 
	 */
	protected void manageShootingAndDriving() {

    	//Drivetrain
        // if button pressed then align, bring shooter up to speed  and Shoot
    	if(m_RobotInterface.GetDriverRightBumper())
    	{
        
        SmartDashboard.putString("Alliance", DriverStation.getInstance().getAlliance().toString());
    	
//    		if(!(Math.abs(previousVisionTurn) == visionAngleMissAligned))
    		{
    			previousVisionTurn = visionAngleMissAligned;
        		visionAlign(visionAngleMissAligned);
    		}
    	}
    	// button not pressed
    	else
    	{
    		
//    		if(m_DriveTrain.isTurnPIDFinished())
//        	{
        		isVisionTurnRunning = false;
        		m_DriveTrain.DisablePIDControlTurn();
//        	}
//    	}
//    	else if(!isVisionTurnRunning)
//    	{
        	arcadeDrive();
//        	arcadeDriveGyroAssist();
    	}
    	m_DriveTrain.WriteDashboardData();
    	
    	//Shooter methods
//    	shoot();
//        public void shoot()
//        {
        	if(m_RobotInterface.GetDriverRightBumper())//if(the Driver's Joystick's right bumper button is pressed)
        	{
        		alignAndShootAutoTrigger();
        	}
        	// Finger is off the AutoShoot button, see if they just want us to align and they will say when to shoot?
        	else if(m_RobotInterface.GetDriverLeftBumper())//if(the Driver's Joystick's right bumper button is pressed)
            {
            	alignAndShootManualTrigger();
            }
        	else
        	{
        		manageLightsForVisionTargetAcquired();

        		m_DriveTrain.DisablePIDControlTurn();
        		m_Shooter.DisablePID();
        		m_Shooter.SetTalonOutput(0);
        	}
	}


	/**
	 * 
	 */
	protected void alignAndShootAutoTrigger() {
		
		turnOnShooterWheel();
		
		// Turn Red lights on so know aligned to target
		if(m_Shooter.ShooterOnTarget())
		{
			m_LightSystem.setRedState(true);
		}
		
		// Turn Blue light on to know Shooter Wheel is up to speed
		if(Math.abs(visionAngleMissAligned)<1 )// if shooter spining at speed and Angle MissAligned is less then 1 Degree
		{
			m_LightSystem.setBlueState(true);
		}
		
		//Shooter is ready to fire, Add the Green ;light indicating we are firing
		if(m_Shooter.ShooterOnTarget() & Math.abs(visionAngleMissAligned)<1 )// if shooter spining at speed and Angle MissAligned is less then 1 Degree
		{
			m_LightSystem.setGreenState(true);
    		m_Indexer.SetTalonOutput(INDEXER_SPEED);
		}
		else{
 	    	m_Indexer.SetTalonOutput(0);
		}

	}

	/**
	 * 
	 */
	protected void turnOnShooterWheel() {
		
    	double lidarDistanceFt = m_RobotSensors.getLidar().getDistanceFt();
        SmartDashboard.putNumber("LidarDistanceFt", lidarDistanceFt);

		double targetRPM;
//        		if(this.haveTarget){
		double distance = 3;// distance in feet
		if(m_RobotSensors.getLidar().getDistanceFt()!=0){
			distance = m_RobotSensors.getLidar().getDistanceFt();
		}
		else
		{
			if(visionDistanceToBoilerTarget==0){
				distance = distance;
			}
			else
			{
				distance = visionDistanceToBoilerTarget;
			}
		}
		targetRPM=calcShooterSpeed(distance);
		
		//Enable the PID Controller for the Shooter Wheel Speed
		m_Shooter.SetShooterSetpoint(targetRPM);
		m_Shooter.EnablePID();
	}

	protected void alignAndShootManualTrigger() {
		turnOnShooterWheel();
		// Turn Red lights on so know aligned to target
		if(m_Shooter.ShooterOnTarget())
		{
			m_LightSystem.setRedState(true);
		}
		// Turn Blue light on to know Shooter Wheel is up to speed
		if(Math.abs(visionAngleMissAligned)<1 )// if shooter spining at speed and Angle MissAligned is less then 1 Degree
		{
			m_LightSystem.setBlueState(true);
		}
		if(m_Shooter.ShooterOnTarget() & Math.abs(visionAngleMissAligned)<1 )// if shooter spining at speed and Angle MissAligned is less then 1 Degree
		{
			//Shooter is ready to fire, Add the Solid Green ;light indicating we are firing
//            		m_LightSystem.setRedState(false);
//            		m_LightSystem.setBlueState(false);
			m_LightSystem.setGreenState(true);
			manageIndexer();
		}

	}
	
	/**
	 * Manages the Light, defaults the color to the alliance we are on. blinks green if vision is in target
	 */
	protected void manageLightsForVisionTargetAcquired() {
		//STOP
		System.out.print("in manageLightsForVisionTargetAcquired visionHasTarget="+visionHasTarget+" BlinkCount="+blinkCounterForHaveTarget);
		// Blink the lights if have a vision target
		if(visionHasTarget){
			if(m_LightSystem.getGreenState())
			{
				blinkCounterForHaveTarget--;            			
			}
			else
			{
				blinkCounterForHaveTarget++;            			
			}
			// Turn lights on or off if hit limit
			if(blinkCounterForHaveTarget > 10)
			{
				m_LightSystem.setGreenState(true);
			}
			if(blinkCounterForHaveTarget < 0)
			{
				m_LightSystem.setGreenState(false);
			}
		}
		else
		{
			m_LightSystem.setGreenState(false);
		}
		
		m_LightSystem.setDefault();
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
    	m_DriveTrain.ArcadeDrive(
    			m_RobotInterface.GetDriverJoystick().getRawAxis(m_RobotInterface.GetDriverForwardPowerAxis()),
    			m_RobotInterface.GetDriverJoystick().getRawAxis(m_RobotInterface.GetDriverRotationPowerAxis()) );
    }

    public void arcadeDriveGyroAssist()
    {
    	m_DriveTrain.ArcadeDriveGyroAssist(m_RobotInterface.GetDriverJoystick(),m_RobotInterface.GetDriverForwardPowerAxis(),m_RobotInterface.GetDriverRotationPowerAxis());
    }

    public void visionAlign(double turn) 
    {
    	SmartDashboard.putNumber("Vision Turn", turn);
    	
    	m_DriveTrain.TurnToAngle(turn);

		isVisionTurnRunning = true;	
    	if(m_DriveTrain.isTurnPIDFinished())
    	{
    		isVisionTurnRunning = false;
    		m_DriveTrain.DisablePIDControlsAll();
    	}
    }
    
    //Shooter methods
    private double calcShooterSpeed(double distanceFromTarget){
    	// From trajectory spreadsheet
    	//y = 11.815x2 + 140.58x + 1348.4 
    	return 11.815*(distanceFromTarget*distanceFromTarget) + 140.58*distanceFromTarget+1348.4;
    }
    

    //Intake methods
    public void manageIntake()
    {
	    if(m_RobotInterface.GetOperatorA())
		{
			m_Intake.SetTalonOutput(INTAKE_SPEED);
		}
	    else
	    {
	    	m_Intake.SetTalonOutput(0);
	    }
    }
    
    //Indexer methods
    public void manageIndexer() 
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
    	if(m_RobotInterface.GetOperatorB())
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
    public void disabledInit() 
    {
    	manageLightsForVisionTargetAcquired();
    } 
    public void disabledPeriodic(){
    	manageLightsForVisionTargetAcquired();
    }
    // Stuff to always be running
    public void robotPeriodic(){ 
    }
}