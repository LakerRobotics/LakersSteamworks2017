package org.usfirst.frc.team5053.robot;

import edu.wpi.first.wpilibj.Joystick;

/**
 * Maps the operator interface devices on the robot.
 * These include but are not limited to the following:
 * Joysticks,
 * Gamepads,
 * Dials,
 * Fancy Buttons,
 * Sliders.
 * 
 * Most importantly, it abstracts the joysticks and buttons away from the main robot project,
  allowing the programmer to call methods to this class itself.
 */


public class RobotInterfaceMap 
{
	public enum joystickType
	{
		LOGITECHGAMEPAD,
		XBOX,
		JOYSTICK
	}
	private final int driverJoystickSlot = 0;
	private final int operatorJoystickSlot = 1;
	
	private int driverButtonX = 2;
	private int driverButtonA = 0;
	private int driverButtonB = 1;
	private int driverButtonY = 3;
	private int driverSelectButton = 6;
	private int driverStartButton = 7;
	private int driverLeftBumper = 4;
	private int driverRightBumper = 5;
	private int driverLeftTrigger = 2;
	private int driverRightTrigger = 3;
	private int driverLeftJoystickButton = 8;
	private int driverRightJoystickButton = 9;
	private int driverLeftY = 1;
	private int driverLeftX = 0;
	private int driverRightX = 4;
	private int driverRightY = 5;
	
	private int operatorButtonX = 2;
	private int operatorButtonA = 0;
	private int operatorButtonB = 1;
	private int operatorButtonY = 3;
	private int operatorSelectButton = 6;
	private int operatorStartButton = 7;
	private int operatorLeftBumper = 4;
	private int operatorRightBumper = 5;
	private int operatorLeftTrigger = 2;
	private int operatorRightTrigger = 3;
	private int operatorLeftJoystickButton = 8;
	private int operatorRightJoystickButton = 9;
	private int operatorLeftY = 1;
	private int operatorLeftX = 0;
	private int operatorRightX = 4;
	private int operatorRightY = 5;
	
	
	private Joystick driverJoystick;
	private Joystick operatorJoystick;
	
	/** Default constructor. Will not initialize any useful abstraction methods (All gamepad calls will return zero)
	 *    Use GetDriverButton(int) and GetOperatorButton(int) to get the direct button number
	 * 
	 */
	public RobotInterfaceMap()
	{
		
		driverJoystick = new Joystick(driverJoystickSlot);
		operatorJoystick = new Joystick(operatorJoystickSlot);
	}
	
	/** Sets the internal button mapping to match the type of controller that
    an operator will be using. 
	 * 
	 */
	public RobotInterfaceMap(joystickType driverType, joystickType operatorType)
	{
		
		switch (driverType)
		{
		case LOGITECHGAMEPAD:
			driverButtonX = 0;
			driverButtonY = 1;
			driverButtonA = 2;
			driverButtonB = 3;
			driverSelectButton = 4;
			driverStartButton = 5;
			driverLeftBumper = 6;
			driverRightBumper = 7;
			driverLeftTrigger = 8;
			driverRightTrigger = 9;
			driverLeftJoystickButton = 10;
			driverRightJoystickButton = 11;
			
			driverLeftY = 1;
			driverLeftX = 2;
			driverRightX = 3;
			driverRightY = 4;
			
			break;
		case XBOX:
			driverButtonX = 0;
			driverButtonY = 1;
			driverButtonA = 2;
			driverButtonB = 3;
			driverSelectButton = 4;
			driverStartButton = 5;
			driverLeftBumper = 6;
			driverRightBumper = 7;
			driverLeftTrigger = 8;
			driverRightTrigger = 9;
			driverLeftJoystickButton = 10;
			driverRightJoystickButton = 11;
			
			driverLeftY = 1;
			driverLeftX = 2;
			driverRightX = 3;
			driverRightY = 4;
			
			break;
		default:
			break;
		}
		
		switch (operatorType)
		{
		case LOGITECHGAMEPAD:
			operatorButtonX = 0;
			operatorButtonY = 1;
			operatorButtonA = 2;
			operatorButtonB = 3;
			operatorSelectButton = 4;
			operatorStartButton = 5;
			operatorLeftBumper = 6;
			operatorRightBumper = 7;
			operatorLeftTrigger = 8;
			operatorRightTrigger = 9;
			operatorLeftJoystickButton = 10;
			operatorRightJoystickButton = 11;
			
			operatorLeftY = 1;
			operatorLeftX = 2;
			operatorRightX = 3;
			operatorRightY = 4;
			
			break;
		case XBOX:
			operatorButtonX = 0;
			operatorButtonY = 1;
			operatorButtonA = 2;
			operatorButtonB = 3;
			operatorSelectButton = 4;
			operatorStartButton = 5;
			operatorLeftBumper = 6;
			operatorRightBumper = 7;
			operatorLeftTrigger = 8;
			operatorRightTrigger = 9;
			operatorLeftJoystickButton = 10;
			operatorRightJoystickButton = 11;
			
			operatorLeftY = 1;
			operatorLeftX = 2;
			operatorRightX = 3;
			operatorRightY = 4;
			
			break;
		default:
			break;
		}
		driverJoystick = new Joystick(driverJoystickSlot);
		operatorJoystick = new Joystick(operatorJoystickSlot);
			
	}

	
	//Driver Controller Abstraction Methods
	public boolean GetDriverButton(int buttonNumber)
	{
		return driverJoystick.getRawButton(buttonNumber);
	}
	public boolean GetDriverX()
	{
		return driverJoystick.getRawButton(driverButtonX);
	}
	public boolean GetDriverY()
	{
		return driverJoystick.getRawButton(driverButtonY);
	}
	public boolean GetDriverA()
	{
		return driverJoystick.getRawButton(driverButtonA);
	}
	public boolean GetDriverB()
	{
		return driverJoystick.getRawButton(driverButtonB);
	}
	public boolean GetDriverSelect()
	{
		return driverJoystick.getRawButton(driverSelectButton);
	}
	public boolean GetDriverStart()
	{
		return driverJoystick.getRawButton(driverStartButton);
	}
	public boolean GetDriverLeftBumper()
	{
		return driverJoystick.getRawButton(driverLeftBumper);
	}
	public boolean GetDriverRightBumper()
	{
		return driverJoystick.getRawButton(driverRightBumper);
	}
	public boolean GetDriverLeftTrigger()
	{
		return driverJoystick.getRawButton(driverLeftTrigger);
	}
	public boolean GetDriverRightTrigger()
	{
		return driverJoystick.getRawButton(driverRightTrigger);
	}
	public boolean GetDriverLeftJoystickButton()
	{
		return driverJoystick.getRawButton(driverLeftJoystickButton);
	}
	public boolean GetDriverRightJoystickButton()
	{
		return driverJoystick.getRawButton(driverRightJoystickButton);
	}
	public double GetDriverLeftX()
	{
		return driverJoystick.getRawAxis(driverLeftX);
	}
	public double GetDriverRightX()
	{
		return driverJoystick.getRawAxis(driverRightX);
	}
	public double GetDriverLeftY()
	{
		return driverJoystick.getRawAxis(driverLeftY);
	}
	public double GetDriverRightY()
	{
		return driverJoystick.getRawAxis(driverRightY);
	}
	public Joystick GetDriverJoystick()
	{
		return driverJoystick;
	}
	
	
	//Operator Controller Abstraction Methods
	
	public boolean GetOperatorButton(int buttonNumber)
	{
		return operatorJoystick.getRawButton(buttonNumber);
	}
	public boolean GetOperatorX()
	{
		return operatorJoystick.getRawButton(operatorButtonX);
	}
	public boolean GetOperatorY()
	{
		return operatorJoystick.getRawButton(operatorButtonY);
	}
	public boolean GetOperatorA()
	{
		return operatorJoystick.getRawButton(operatorButtonA);
	}
	public boolean GetOperatorB()
	{
		return operatorJoystick.getRawButton(operatorButtonB);
	}
	public boolean GetOperatorSelect()
	{
		return operatorJoystick.getRawButton(operatorSelectButton);
	}
	public boolean GetOperatorStart()
	{
		return operatorJoystick.getRawButton(operatorStartButton);
	}
	public boolean GetOperatorLeftBumper()
	{
		return operatorJoystick.getRawButton(operatorLeftBumper);
	}
	public boolean GetOperatorRightBumper()
	{
		return operatorJoystick.getRawButton(operatorRightBumper);
	}
	public boolean GetOperatorLeftTrigger()
	{
		return operatorJoystick.getRawButton(operatorLeftTrigger);
	}
	public boolean GetOperatorRightTrigger()
	{
		return operatorJoystick.getRawButton(operatorRightTrigger);
	}
	public boolean GetOperatorLeftJoystickButton()
	{
		return operatorJoystick.getRawButton(operatorLeftJoystickButton);
	}
	public boolean GetOperatorRightJoystickButton()
	{
		return operatorJoystick.getRawButton(operatorRightJoystickButton);
	}
	public double GetOperatorLeftX()
	{
		return operatorJoystick.getRawAxis(operatorLeftX);
	}
	public double GetOperatorRightX()
	{
		return operatorJoystick.getRawAxis(operatorRightX);
	}
	public double GetOperatorLeftY()
	{
		return operatorJoystick.getRawAxis(operatorLeftY);
	}
	public double GetOperatorRightY()
	{
		return operatorJoystick.getRawAxis(operatorRightY);
	}
	public Joystick GetOperatorJoystick()
	{
		return operatorJoystick;
	}

}
