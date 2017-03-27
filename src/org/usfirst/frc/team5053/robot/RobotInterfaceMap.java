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
	public enum JoystickType
	{
		LOGITECHGAMEPAD,
		XBOX,
		JOYSTICK
	}
	private final int driverJoystickSlot = 0;
	private final int operatorJoystickSlot = 1;
	
	private int driverButtonX = 0;
	private int driverButtonA = 0;
	private int driverButtonB = 0;
	private int driverButtonY = 0;
	private int driverSelectButton = 0;
	private int driverStartButton = 0;
	private int driverLeftBumper = 0;
	private int driverRightBumper = 0;
	private int driverLeftTrigger = 0;
	private int driverRightTrigger = 0;
	private int driverLeftJoystickButton = 0;
	private int driverRightJoystickButton = 0;
	private int driverLeftY = 0;
	private int driverLeftX = 0;
	private int driverRightX = 0;
	private int driverRightY = 0;
	
	private int operatorButtonX = 0;
	private int operatorButtonA = 0;
	private int operatorButtonB = 0;
	private int operatorButtonY = 0;
	private int operatorSelectButton = 0;
	private int operatorStartButton = 0;
	private int operatorLeftBumper = 0;
	private int operatorRightBumper = 0;
	private int operatorLeftTrigger = 0;
	private int operatorRightTrigger = 0;
	private int operatorLeftJoystickButton = 0;
	private int operatorRightJoystickButton = 0;
	private int operatorLeftY = 0;
	private int operatorLeftX = 0;
	private int operatorRightX = 0;
	private int operatorRightY = 0;
	
	private Joystick driverJoystick;
	private Joystick operatorJoystick;
	
	/** Default constructor. Will detect if the plugged in controller is xbox  and adjust setting accordingly.
	 * 
	 */
	public RobotInterfaceMap(JoystickType driver, JoystickType operator)
	{
		driverJoystick = new Joystick(driverJoystickSlot);
		operatorJoystick = new Joystick(operatorJoystickSlot);		
		
		if (driver.equals(JoystickType.XBOX)){
			//XBOX driver
			driverButtonA = 1;
			driverButtonB = 2;
			driverButtonX = 3;
			driverButtonY = 4;
			driverLeftBumper = 5;
			driverRightBumper = 6;
			driverSelectButton = 7;
			driverStartButton = 8;
			driverLeftJoystickButton = 9;
			driverRightJoystickButton = 10;

			driverLeftX = 0;
			driverLeftY = 1;
			driverLeftTrigger = 2;
			driverRightTrigger = 3;
			driverRightX = 4;
			driverRightY = 5;
		}
		else
		{
			//JOYSTICK driver
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
		}
		
		if (operator.equals(JoystickType.XBOX))
		{
			//XBOX operator
			operatorButtonA = 1;
			operatorButtonB = 2;
			operatorButtonX = 3;
			operatorButtonY = 4;
			operatorLeftBumper = 5;
			operatorRightBumper = 6;
			operatorSelectButton = 7;
			operatorStartButton = 8;
			operatorLeftJoystickButton = 9;
			operatorRightJoystickButton = 10;

			operatorLeftX = 0;
			operatorLeftY = 1;
			operatorLeftTrigger = 2;
			operatorRightTrigger = 3;
			operatorRightX = 4;
			operatorRightY = 5;
			
		}
		else
		{
			//JOYSTICK operator
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
		}	
	}

	
	//Driver Controller Abstraction Methods
	public boolean GetDriverButton(int buttonNumber){return driverJoystick.getRawButton(buttonNumber);}
	public boolean GetDriverX(){                     return driverJoystick.getRawButton(driverButtonX);}
	public boolean GetDriverY(){                     return driverJoystick.getRawButton(driverButtonY);}
	public boolean GetDriverA(){                     return driverJoystick.getRawButton(driverButtonA);}
	public boolean GetDriverB(){                     return driverJoystick.getRawButton(driverButtonB);}
	public boolean GetDriverSelect(){                return driverJoystick.getRawButton(driverSelectButton);}
	public boolean GetDriverStart(){                 return driverJoystick.getRawButton(driverStartButton);}
	public boolean GetDriverLeftBumper(){            return driverJoystick.getRawButton(driverLeftBumper);}
	public boolean GetDriverRightBumper(){           return driverJoystick.getRawButton(driverRightBumper);}
	public boolean GetDriverLeftJoystickButton(){    return driverJoystick.getRawButton(driverLeftJoystickButton);}
	public boolean GetDriverRightJoystickButton(){   return driverJoystick.getRawButton(driverRightJoystickButton);}
	public double GetDriverLeftX(){                  return driverJoystick.getRawAxis(driverLeftX);}
	public double GetDriverRightX(){                 return driverJoystick.getRawAxis(driverRightX);}
	public double GetDriverLeftY(){                  return driverJoystick.getRawAxis(driverLeftY);}
	public double GetDriverRightY(){                 return driverJoystick.getRawAxis(driverRightY);}
	public boolean GetDriverLeftTrigger(){            return driverJoystick.getRawAxis(driverLeftTrigger) > 0.0;}
	public boolean GetDriverRightTrigger(){			 return driverJoystick.getRawAxis(driverRightTrigger) > 0.0;}
	public Joystick GetDriverJoystick(){             return driverJoystick;}
	
	
	//Operator Controller Abstraction Methods
	
	public boolean GetOperatorButton(int buttonNumber){return operatorJoystick.getRawButton(buttonNumber);}
	public boolean GetOperatorX(){                     return operatorJoystick.getRawButton(operatorButtonX);}
	public boolean GetOperatorY(){                     return operatorJoystick.getRawButton(operatorButtonY);}
	public boolean GetOperatorA(){                     return operatorJoystick.getRawButton(operatorButtonA);}
	public boolean GetOperatorB(){                     return operatorJoystick.getRawButton(operatorButtonB);}
	public boolean GetOperatorSelect(){                return operatorJoystick.getRawButton(operatorSelectButton);}
	public boolean GetOperatorStart(){                 return operatorJoystick.getRawButton(operatorStartButton);}
	public boolean GetOperatorLeftBumper(){            return operatorJoystick.getRawButton(operatorLeftBumper);}
	public boolean GetOperatorRightBumper(){           return operatorJoystick.getRawButton(operatorRightBumper);}
	public boolean GetOperatorLeftTrigger(){           return operatorJoystick.getRawButton(operatorLeftTrigger);}
	public boolean GetOperatorRightTrigger(){          return operatorJoystick.getRawButton(operatorRightTrigger);}
	public boolean GetOperatorLeftJoystickButton(){    return operatorJoystick.getRawButton(operatorLeftJoystickButton);}
	public boolean GetOperatorRightJoystickButton(){   return operatorJoystick.getRawButton(operatorRightJoystickButton);}
	public double GetOperatorLeftX(){                  return operatorJoystick.getRawAxis(operatorLeftX);}
	public double GetOperatorRightX(){                 return operatorJoystick.getRawAxis(operatorRightX);}
	public double GetOperatorLeftY(){                  return operatorJoystick.getRawAxis(operatorLeftY);}
	public double GetOperatorRightY(){                 return operatorJoystick.getRawAxis(operatorRightY);}
	public Joystick GetOperatorJoystick(){             return operatorJoystick;}

}
