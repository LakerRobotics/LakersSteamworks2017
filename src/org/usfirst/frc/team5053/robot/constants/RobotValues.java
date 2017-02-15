package org.usfirst.frc.team5053.robot.constants;
/*
 * This is a base orginzational class that provides the pattern for a class that holds all the robot specific setting
 * it is expected there will be a LilGeek, Machinac and possibly Rudy implementation of this class
 */
public abstract class RobotValues {

		public static double getLeftEncoderDistancePerPulse(){return 0.0;};//converts click to distance in inches
		public static double getRightEncoderDistancePerPulse(){return 0.0;};//converts click to distance in inches
		public static double getShooterEncoderDistancePerPulse(){return 0.0;};//converts click to RPM

		// Drive Train motion Control distance
		public static double getPidDist_P(){return 0.0;};
		public static double getPidDist_I(){return 0.0;};
		public static double getPidDist_D(){return 0.0;};
		public static double getPidDist_AbsoluteTolerance(){return 0.0;};
		
}
