package org.usfirst.frc.team5053.robot.sensors;

// Code from 254 Cheesy Poofs

// Modified 2017/2/12 by Richard Topolewski, changed return distance to return ft (had been returning meters)

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.Timer;

import java.util.TimerTask;

public class LidarLiteSensor {
    private I2C mI2C;
    private byte[] mDistance;
    private java.util.Timer mUpdater;
    private boolean mHasSignal;

    private final static int LIDAR_ADDR = 0x62;
//    private final static int LIDAR_ADDR = 0xC4;//8"bit"I2C"address"is:"0xC4"write,"0xC5"read (v1 user manual)
//    private final static int LIDAR_ADDR = 0xC5;//8"bit"I2C"address"is:"0xC4"write,"0xC5"read (v1 user manual)
    
    private final static int LIDAR_CONFIG_REGISTER = 0x00;
    private final static int LIDAR_CONFIG_START_TAKING_MESUREMENTS = 0x04;          // Value to initiate ranging.
    private final static int LIDAR_REGISTER_HARDWARE_VER = 0x41;
    private final static int LIDAR_DISTANCE_REGISTER = 0x8f;
    
    //from v1 User manual:
    // The"Sensor"module"has"a"7bit" slave "address" with a default value of "0x62" in hexadecimal notation.
    // The effective 8"bit"I2C"address"is:"0xC4"write,"0xC5"read (v1 user manual)
    // I2C WPI using expects 7bit address (changed in 2015 http://wpilib.screenstepslive.com/s/4485/m/13809/l/599750-archive-c-java-porting-guide-2014-to-2015)

    public LidarLiteSensor() {
        mI2C = new I2C(I2C.Port.kMXP, LIDAR_ADDR);
        mDistance = new byte[2];
        mUpdater = new java.util.Timer();
        mHasSignal = false;
    }
    
    public boolean isWorking(){
    	byte[] expected = new byte[1];
    	expected[0]= 0x01;
    	boolean sensorWorks = mI2C.verifySensor(0x41, 1, expected);
        System.out.println("I2C.verifySensor(0x41,1,0x01)="+sensorWorks);
    	return sensorWorks;
    }

    /**
     * @return Distance in feet
     */
    public double getDistance() {
        int distCm = (int) Integer.toUnsignedLong(mDistance[0] << 8) + Byte.toUnsignedInt(mDistance[1]);
        double distMeters = distCm / 100.0;
        // 1 meter = 3.28084ft
        return distMeters * 3.28084;
    }

    /**
     * @return true iff the sensor successfully provided data last loop
     */
    public boolean hasSignal() {
        return mHasSignal;
    }

    /**
     * Start 10Hz polling
     */
    public void start() {
        start(100);
    }
/*
 * scan all registers see if get something besides all zeros
 * THis is a Debugging function, to be removed once things are working
 */
    public void scan(){
    	
    	boolean writeAborted = mI2C.write(LIDAR_CONFIG_REGISTER, LIDAR_CONFIG_START_TAKING_MESUREMENTS);
        System.out.print("I2C.write("+LIDAR_CONFIG_REGISTER+","+LIDAR_CONFIG_START_TAKING_MESUREMENTS+"):aborted="+writeAborted+ " ");
        if (writeAborted) {
            // the write failed to ack
            mHasSignal = false;
            return;
        }else{
        	mHasSignal=true;
        }
        
        Timer.delay(0.04); // Delay for measurement to be taken
        byte[] returnBytes = new byte[1];
        for(int i=0;i<=104;i++){
             boolean readReturned = mI2C.read(i, 1, returnBytes);
            System.out.println("I2C.read("+i+",1,returnBytes)="+readReturned+" returns returnBytes[0]="+returnBytes[0]+ " ");
            Timer.delay(0.005); // Delay to prevent over polling
        }

    }
    /**
     * Start polling for period in milliseconds
     */
    public void start(int period) {

    	scan();
    	
    	mDistance[0]=1; 
	    mDistance[1]=2;
    	System.out.print("I2C mDistance="+mDistance+ " "); 
    	System.out.print("I2C mDistance[0]="+mDistance[0]+ " "); 
    	System.out.print("I2C mDistance[1]="+mDistance[1]+ " ");
    	
        TimerTask task = new TimerTask() {
            @Override
            public void run() {
                update();
            }
        };
        mUpdater.scheduleAtFixedRate(task, 0, period);
    }

    public void stop() {
        mUpdater.cancel();
        mUpdater = new java.util.Timer();
    }

    private void update() {
    	boolean writeAborted = mI2C.write(LIDAR_CONFIG_REGISTER, LIDAR_CONFIG_START_TAKING_MESUREMENTS);
 //       System.out.print("I2C.write("+LIDAR_CONFIG_REGISTER+","+LIDAR_CONFIG_START_TAKING_MESUREMENTS+"):aborted="+writeAborted+ " ");
//        if (writeAborted) {
//            // the write failed to ack
//            mHasSignal = false;
//            return;
//        }else{
//        	mHasSignal=true;
//        }
        
        Timer.delay(0.04); // Delay for measurement to be taken
        
        boolean readAborted = mI2C.read(LIDAR_DISTANCE_REGISTER, 2, mDistance);
 //       System.out.print("I2C.read("+LIDAR_DISTANCE_REGISTER+",2,"+mDistance+"):aborted="+readAborted+ " ");
 
//        if (readAborted) {
//            // the read failed
//            mHasSignal = false;
//            return;
//        }
        mHasSignal = true;
//        int distCm = (int) Integer.toUnsignedLong(mDistance[0] << 8) + Byte.toUnsignedInt(mDistance[1]);
 //       System.out.print("I2C.read("+LIDAR_REGISTER_HARDWARE_VER+",1,"+mDistance+"):aborted="+readAborted+ " ");

//        System.out.print("I2C Hardware ver mDistance[0]="+mDistance[0]+ " "); 
//     	System.out.print("I2C mDistance[0]="+mDistance[0]+ " "); 
//    	System.out.print("I2C mDistance[1]="+mDistance[1]+ " ");
//    	System.out.print("Distance="+distCm+" cm  ");
        System.out.println("Distance="+getDistance()+" ft  ");
 

        Timer.delay(0.005); // Delay to prevent over polling
    }
}

