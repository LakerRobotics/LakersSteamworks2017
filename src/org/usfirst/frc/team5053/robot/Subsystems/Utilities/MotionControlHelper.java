package org.usfirst.frc.team5053.robot.Subsystems.Utilities;

import java.lang.reflect.Method;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GyroBase;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * 
 * @author richard.topolewski
 *
 */
public class MotionControlHelper {
	double m_targetDistance         = 0.0d; // in distance units for example inches or Degrees of rotation
	double m_rampUpRampDownDistance = 0.0d; // in distance units for example inches or Degrees of roation
    double m_runningSpeed           = 0.0d; // distance (e.g. inchs or Degrees of rotation) over seconds	
	
    double m_currentMeasuredDistance  = 0.0d;
    double m_initialMeasuredDistance  = 0.0d;
 
   	public double percentDeadZoneOverride = 0.50;//enter portion of 1 (e.g. .1 for 10%)
   
	PIDOutput m_output;
	PIDSource m_source;
	
	PIDController regularPIDControl;
	   	
	/**
     * This helper class just takes a target distance and will provide a motion control speed to get to that target
     * @param aTargetDistance   Where we are trying to get to
     * @param aRampUpRampDownDistance  Allows control of how fast we accelerate and decelerate
     * @param aRunningSpeed   the speed we want to travel most of the time, except for ramp up and ramp down
     * @param aInitialMeasuredDistance  so we know where we started from for the ramp up
     */
    public MotionControlHelper(double targetDistance, double rampUpRampDownDistance, double runningSpeed, 
    		            double initialMeasuredDistance,
    		            PIDSource source, PIDOutput output){
    	m_targetDistance          = targetDistance;
    	m_rampUpRampDownDistance  = Math.abs(rampUpRampDownDistance);
    	m_runningSpeed            = runningSpeed;
    	m_initialMeasuredDistance = Math.abs(initialMeasuredDistance);
    	m_output = output;
    	m_source = source;

    	}

    protected PIDController getRegularPIDControl() {
		return regularPIDControl;
	}

	protected void setRegularPIDControl(PIDController regularPIDControl) {
		this.regularPIDControl = regularPIDControl;
	}
	
	public void setTargetDistance(double new_targetDistance){
		m_targetDistance = new_targetDistance;
	}


    public PIDOutput getM_output() {
		return m_output;
	}

    /**
     * This returns the PIDSource wrapped so when called by the PIDController the motionControlHelper can
     * adjust the target rate that the PIDController is trying to achieve
     * @return
     */
	public PIDSource getM_source() {
		return new wrapPIDInput(this, m_source);
	}

	/**
     * Given the currentPosition (i.e. distance) this will return the current target speed 
     * @param currentMeasuredDistance // the current position, in distance units for example inches or Degrees-of-rotation
     * @return
     */
    public double getTargetSpeed(double currentMeasuredDistance){
       double targetSpeed = 0.0d;       
       
       // get the motors going in the right direction
       double gapEnd = m_targetDistance-currentMeasuredDistance;
       if(gapEnd == 0) 
       {
    	   targetSpeed = 0;
       }
       else 
       {
    	   targetSpeed = (gapEnd/Math.abs(gapEnd)) * m_runningSpeed; // This just applied +1 or -1 to get the sign right
       }
       
       // Calculate the reduction to the speed if at the start
       double percentRampUp;
       double gapStart = currentMeasuredDistance-m_initialMeasuredDistance;
       if( Math.abs(gapStart) > m_rampUpRampDownDistance)
       {
    	   // We are outside of the rampUp zone 
    	   percentRampUp = 1; //100%
       }
       else
       {
    	   // Are we right on top of the start point, if so, don't set motor to zero but some minimum number to get things to move
           if( Math.abs(gapStart) < m_rampUpRampDownDistance*(percentDeadZoneOverride))
           {
    	      percentRampUp = percentDeadZoneOverride ; //just to make sure it does not stay stuck at the start 
           }
           else
           {
               percentRampUp = Math.abs(gapStart)/m_rampUpRampDownDistance;
           }
       }
       
       // Calculate reduction to the speed if we are at the end
       double percentRampDown = Math.abs(gapEnd)/m_rampUpRampDownDistance;
       if (Math.abs(percentRampDown)>1)  percentRampDown = 1; // limit percent to 100%

      //Apply any speed reductions based on rampUp or rampDown.
//       System.out.println("fromStart="+gapStart+"("+percentRampUp+")   fromEnd="+gapEnd+"("+percentRampDown+")");
// Removed this 4/12/2016 seemed to cause problem, if close the end, the start dead overide would kick in and make it overshoot the target       
//       if(Math.abs(gapStart)<Math.abs(gapEnd)){
//    	   targetSpeed = percentRampUp * targetSpeed;
//       }
//       else{
       // If we are near the end, then ramp down
       if(Math.abs(gapEnd) < m_rampUpRampDownDistance) 
       {
    	   targetSpeed = percentRampDown * targetSpeed;
       }
      // System.out.println("targetSpeed="+targetSpeed);
       SmartDashboard.putNumber("targetSpeed",targetSpeed);
       
       SmartDashboard.putNumber("getTargetSpeed MotionControlHelper", targetSpeed);
       System.out.println("getTargetSpeed MotionControlHelper: " + Double.toString(targetSpeed));
       return targetSpeed;
    }
    
	/**
	 * The PIDSource we to ensure it is
	 * returning rate to the PIDController
	 */
	public void ensureSourceProvidesRate() {
		
		m_source.setPIDSourceType(PIDSourceType.kRate);

	}
    
    /**
     * Read the input(i.e. position) and calculate the speed for this position and put that in as the setPoint
     */
    protected void adjustTargetSpeed() throws Exception {
    	//using the current measurement get the desired rate (i.e. speed)
    	
    	ensureSourceProvidesRate();
    	double currentSpeed = m_source.pidGet();
    	
    	double targetSpeed = getTargetSpeed(this.getMeasurment());
    	SmartDashboard.putNumber("MotionControlHelper.adjustTargetSpeed Measurement", this.getMeasurment());
    	//System.out.println("MotionControlHelper.adjustTargetSpeed targetSpeed="+targetSpeed + "  ActualSpeed="+currentSpeed  + "targetPosition="+ this.m_targetDistance+"    Current Postion="+this.getMeasurment());
    	this.getRegularPIDControl().setSetpoint(targetSpeed);
    	SmartDashboard.putNumber("MotionControlHelper.adjustTargetSpeed targetSpeed", targetSpeed);
    	
    	ensureSourceProvidesRate();
    	SmartDashboard.putNumber("MotionControlHelper.adjustTargetSpeed Gyro Rate", m_source.pidGet());
    	// now that we have the speed set properly lets call the PID control and have it adjust the PIDInput (e.g. the motor power) to get closer to the desired speed.
    	//TODO need to access the inner class PIDTask and override to call calculatesSetup then then calculate()
       	//super.calculate();
    }

	public double getMeasurment() {
		// Store away the what the Source is to return
		PIDSourceType tempType = m_source.getPIDSourceType();
		// Switch to report on where were at, and get where we are at
		m_source.setPIDSourceType(PIDSourceType.kDisplacement);
		double returnValue =  m_source.pidGet();
		// revert PIDSource back to what it was reporting before (either Rate or Displacement)
		m_source.setPIDSourceType(tempType);
		// Actually return the measurement (i.e. displacement or location)
		return returnValue;
	}	

    
    private class wrapPIDInput implements PIDSource {

        private MotionControlHelper m_MCHelper;
        private PIDSource m_source; 

        public wrapPIDInput(MotionControlHelper motionControlHelper, PIDSource source) {
            if (motionControlHelper == null) {
                throw new NullPointerException("Given MotionControlPIDController was null");
            }
            else{
                m_MCHelper = motionControlHelper;            	
            }
            
            if (source == null){
                throw new NullPointerException("Given PIDSource was null");
            }
            else{
                m_source = source;
            }
        }
        
		@Override
        public double pidGet(){
        	// have the controller set the target speed,
        	//TODO have WPI redo the PIDController so the calculate() method is protected so we wouldn't have to do this hack 
			//  if it were protected then we could override calculate() method and allow the target speed to be set ahead of calculation the new PID output
			try{
				m_MCHelper.adjustTargetSpeed();
			}
			catch (Exception e){
				System.out.println("MotionControl PIDSource BAD, likley not Gyro or Encoder or missing getMeasurement()");
				System.out.println(e);
			}
			// call the real PIDSource
        	return m_source.pidGet();
        }

		@Override
		public void setPIDSourceType(PIDSourceType pidSource) {
			m_source.setPIDSourceType(pidSource);
//			System.out.println("ERROR MotionControlHelper.setPIDSourceType() CALL BEING IGNORED because this Motion control controls Rate");
			
		}

		@Override
		public PIDSourceType getPIDSourceType() {
			return m_source.getPIDSourceType();
			//return PIDSourceType.kRate;
			//return m_pidSource;
		}

    }
    
    
    //Time
}
