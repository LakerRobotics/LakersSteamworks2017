package org.usfirst.frc.team5053.robot.Subsystems;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.FeedbackDeviceStatus;
import com.ctre.CANTalon.TalonControlMode;
import com.ctre.CANTalon.VelocityMeasurementPeriod;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter implements Subsystem {

	private CANTalon m_Shooter;
	//private Encoder m_Encoder;
	//private PIDController m_PID;
	
//config through RoboRIO webpage	private double KP = 0.0001;
//config through RoboRIO webpage	private double KI = 0.0005;
//config through RoboRIO webpage	private double KD = 0.0;
	
	private double TOLERANCE_RPM = 10;
	
	boolean debug = true;
	
	public Shooter(CANTalon shooterTalon) {
		m_Shooter = shooterTalon;
		//m_Encoder = shooterEncoder;
		
		m_Shooter.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);// Think this sets us up to control RPM so don't need to set RPM per tick, test in the RoboRio control panel self test
		//m_Shooter.configEncoderCodesPerRev((60.0d/1024.0d)*(72.0d/24.0d));//It wants INT????
		m_Shooter.reverseSensor(false);
		//m_Shooter.reverseOutput(true);
		m_Shooter.changeControlMode(TalonControlMode.Speed);
		
		m_Shooter.configNominalOutputVoltage(0, 0);
		m_Shooter.configPeakOutputVoltage(12.7,-12.7);
		
		// Check Sensor Health
		FeedbackDeviceStatus status = m_Shooter.isSensorPresent(FeedbackDevice.CtreMagEncoder_Relative);
		if(debug) if(status == FeedbackDeviceStatus.FeedbackStatusPresent){ System.out.println ("The TalonSRX detects the Mag Encoder Sensor");} 
		
		m_Shooter.SetVelocityMeasurementPeriod(VelocityMeasurementPeriod.Period_20Ms); // think this is best though not as smooth Period_20Ms
		m_Shooter.SetVelocityMeasurementWindow(32);//think this is best though not as smooth 32
		
//		m_Shooter.setPID(KP,KI,KD);//(p, i, d); (0.3,0.001,0.0);
		//more complex, we don't need/not used to.		m_Shooter.setPID(0.01,0.001,0,0,0,0.0,0);//(p, i, d, f, izone, closeLoopRampRate, profile); (0.3,0.001,0.0);
		//more complex, we don't need.		m_Shooter.setCloseLoopRampRate(0.0);// how fast the voltage going to the motor can change, in this case we specify 0, so no limit
		//more complex, we don't need.		m_Shooter.setIZone(0);// can define multiple sets of PID settings, we are just using the zero PIDS setting

		//m_Shooter.setAllowableClosedLoopErr(100);// should allow 100 RPM error and still say it is on target
		
	}
	
	public void EnablePID() {
			m_Shooter.enableControl();
	}
	public void DisablePID() {
			m_Shooter.disableControl();
	}
	public boolean isPIDEnabled() {
		return m_Shooter.isControlEnabled();
	}
	public boolean ShooterOnTarget() {		
		if(Math.abs(m_Shooter.getEncVelocity()-m_Shooter.getSpeed()) < TOLERANCE_RPM)
		{
			return true;
		}
		else
		{
			return false;
		}
	}
	public void SetShooterSetpoint(double speed) {
		m_Shooter.set(speed);

		//talon.SetVelocityMeasurementPeriod(VelocityMeasurementPeriod.Period_100Ms);
		//talon.SetVelocityMeasurementWindow(64); 
		//m_PID.setSetpoint(speed);
	}
	public void SetTalonOutput(double speed) {
		m_Shooter.set(speed);
	}
	public void WriteDashboardData() {
		SmartDashboard.putNumber("Shooter SRX Setpoint (m_Shooter.getSetPoint())", m_Shooter.getSetpoint());
		SmartDashboard.putNumber("Shooter SRX RPM (m_Shooter.get())", m_Shooter.getSpeed());
		SmartDashboard.putNumber("Shooter SRX Velocity (m_Shooter.getEncVelocity)", m_Shooter.getEncVelocity());
		SmartDashboard.putNumber("Shooter SRX OutputVoltage", m_Shooter.getOutputVoltage());
		SmartDashboard.putNumber("Shooter SRX OutputCurrent", m_Shooter.getOutputCurrent());
		
	}
}