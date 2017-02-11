package org.usfirst.frc.team5053.robot.Subsystems;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DigitalOutput;

public class LightSystem implements Subsystem {
	
	private DigitalOutput m_Red;
	private DigitalOutput m_Blue;
	private DigitalOutput m_Green;
	
	private Alliance m_Alliance;
	
	public LightSystem(DigitalOutput red, DigitalOutput blue, DigitalOutput green, Alliance alliance)
	{
		m_Red = red;
		m_Blue = blue;
		m_Green = green;
		
		m_Alliance = alliance;
	}
	
	public void setRedState(boolean isOn)
	{
		m_Red.set(isOn);
	}
	public void setBlueState(boolean isOn)
	{
		m_Blue.set(isOn);
	}
	public void setGreenState(boolean isOn)
	{
		m_Green.set(isOn);
	}
	public void setDefault() 
	{
		if(m_Alliance.equals(Alliance.Red))
    	{
    		m_Red.set(true);
    		
    		m_Blue.set(false);
    		m_Green.set(false);
    	}
    	else
    	{
    		m_Blue.set(true);
    		
    		m_Red.set(false);
    		m_Green.set(false);
    	}
	}
	@Override
	public void WriteDashboardData() {
		// TODO Auto-generated method stub
		
	}
	
}
