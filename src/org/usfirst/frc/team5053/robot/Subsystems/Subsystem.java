package org.usfirst.frc.team5053.robot.Subsystems;

import java.util.HashMap;

public interface Subsystem
{
	public HashMap<String, Double> GetDashboardData();
	
	/*
	 *  Serves as a template that includes all methods a subsystem should have
	 *  Mostly used for PID controller wrapping and debugging output functions
	 */
}
