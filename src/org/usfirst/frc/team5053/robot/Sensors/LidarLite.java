package org.usfirst.frc.team5053.robot.Sensors;

// Code from 254 Cheesy Poofs

// Modified 2017/2/12 by Richard Topolewski, changed return distance to return ft (had been returning meters)

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.Timer;

import java.util.TimerTask;

public class LidarLite {
	
    private I2C m_I2C;
    private byte[] m_Distance;
    private java.util.Timer m_Updater;
    private boolean m_HasSignal;

    private final int LIDAR_ADDR = 0x62;
    private final int LIDAR_CONFIG_REGISTER = 0x00;
    private final int LIDAR_CONFIG_START_TAKING_MESUREMENTS = 0x04; // Value to initiate ranging.
    private final int LIDAR_REGISTER_HARDWARE_VER = 0x41;
    private final int LIDAR_DISTANCE_REGISTER = 0x8f;
    
    //from v1 User manual:
    // The"Sensor"module"has"a"7bit" slave "address" with a default value of "0x62" in hexadecimal notation.
    // The effective 8"bit"I2C"address"is:"0xC4"write,"0xC5"read (v1 user manual)
    // I2C WPI using expects 7bit address (changed in 2015 http://wpilib.screenstepslive.com/s/4485/m/13809/l/599750-archive-c-java-porting-guide-2014-to-2015)

    public LidarLite() {

        m_I2C = new I2C(I2C.Port.kMXP, LIDAR_ADDR);
        m_Distance = new byte[2];
        m_Updater = new java.util.Timer();
        m_HasSignal = false;

    }
    
    public boolean isWorking(){
    	byte[] expected = new byte[1];
    	expected[0]= 0x01;
    	boolean sensorWorks = m_I2C.verifySensor(0x41, 1, expected);
        System.out.println("I2C.verifySensor(0x41,1,0x01)="+sensorWorks);
    	return sensorWorks;
    }

    /**
     * @return Distance in feet
     */
    public double getDistanceFt() 
    {
        int distCm = (int) Integer.toUnsignedLong(m_Distance[0] << 8) + Byte.toUnsignedInt(m_Distance[1]);
        double distMeters = distCm / 100.0;
        // 1 meter = 3.28084ft
        return distMeters * 3.28084;
    }
    
    public double getDistanceCm()
    {
    	return Integer.toUnsignedLong(m_Distance[0] << 8) + Byte.toUnsignedInt(m_Distance[1]);
    }

    /**
     * @return true iff the sensor successfully provided data last loop
     */
    public boolean hasSignal() 
    {
        return m_HasSignal;
    }

    /**
     * Start 10Hz polling
     */
    public void start() 
    {
        start(100);
    }
/*
 * scan all registers see if get something besides all zeros
 * THis is a Debugging function, to be removed once things are working
 */
    public void scan(){
    	
    	boolean writeAborted = m_I2C.write(LIDAR_CONFIG_REGISTER, LIDAR_CONFIG_START_TAKING_MESUREMENTS);
        
    	if (writeAborted) 
    	{
            // the write failed to ack
            m_HasSignal = false;
            return;
        }
    	else
    	{
        	m_HasSignal=true;
        }
        
        Timer.delay(0.04); // Delay for measurement to be taken
        byte[] returnBytes = new byte[1];
        for(int i=0;i<=104;i++)
        {
             boolean readReturned = m_I2C.read(i, 1, returnBytes);
            Timer.delay(0.005); // Delay to prevent over polling
        }

    }
    /**
     * Start polling for period in milliseconds
     */
    public void start(int period) {

    	scan();
    	
    	m_Distance[0]=1; 
	    m_Distance[1]=2;
    	
        TimerTask task = new TimerTask() {
            @Override
            public void run() {
                update();
            }
        };
        m_Updater.scheduleAtFixedRate(task, 0, period);
    }

    public void stop() {
        m_Updater.cancel();
        m_Updater = new java.util.Timer();
    }

    private void update() {
    	m_I2C.write(LIDAR_CONFIG_REGISTER, LIDAR_CONFIG_START_TAKING_MESUREMENTS);
        Timer.delay(0.04); // Delay for measurement to be taken
        m_I2C.read(LIDAR_DISTANCE_REGISTER, 2, m_Distance);
        Timer.delay(0.005); // Delay to prevent over polling
    }
}