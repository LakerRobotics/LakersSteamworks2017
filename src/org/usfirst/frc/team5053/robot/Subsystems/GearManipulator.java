package org.usfirst.frc.team5053.robot.Subsystems;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class GearManipulator implements Subsystem {
	
	private SpeedController m_Arm;
	private Encoder m_ArmEncoder;
	private PIDController m_ArmPID;
	
	private SpeedController m_GearManipulator;
	private Encoder m_GearManipulatorEncoder;
	private PIDController m_GearManipulatorPID;

	private double ARM_KP = 15.0;
	private double ARM_KI = 0.05;
	private double ARM_KD = 0.0;
	
	private final double GEAR_KP = 0.0;
	private final double GEAR_KI = 0.0;
	private final double GEAR_KD = 0.0;
	
	private double HIGHEST_LIMIT = 0.0;
	private double LOWEST_LIMIT = 0.0;
	
	public GearManipulator(SpeedController armTalon, SpeedController gearTalon, Encoder armEncoder, Encoder gearEncoder) {
		m_Arm = armTalon;
		m_ArmEncoder = armEncoder;
		m_ArmEncoder.setPIDSourceType(PIDSourceType.kRate);
		m_ArmPID = new PIDController(ARM_KP, ARM_KI, ARM_KD, m_ArmEncoder, m_Arm);
		
		m_GearManipulator = gearTalon;
		m_GearManipulatorEncoder = gearEncoder;
		m_GearManipulatorEncoder.setPIDSourceType(PIDSourceType.kRate);
		m_GearManipulatorPID = new PIDController(GEAR_KP, GEAR_KI, GEAR_KD, m_GearManipulatorEncoder, m_GearManipulator);
	}
	
	public void EnablePID() {
		if(!m_ArmPID.isEnabled())
		{
			m_ArmPID.enable();
		}
	}
	
	public void DisablePID() {
		if(m_ArmPID.isEnabled())
		{
			m_ArmPID.disable();
		}
	}
	
	public boolean isPIDEnabled() {
		return m_ArmPID.isEnabled();
	}
	
	public double GetArmPosition() {
		return m_ArmEncoder.getDistance();
	}
	
	public void SetTargetPosition(double target) {
		if(target < HIGHEST_LIMIT && target > LOWEST_LIMIT) 
		{
			m_ArmPID.setSetpoint(target);
		}
	}
	
	public void SetTalonOutput(double speed) { 
		m_Arm.set(speed);
	}
	
	public void WriteDashboardData() {
		SmartDashboard.putNumber("Arm Encoder", GetArmPosition());
	}
}
