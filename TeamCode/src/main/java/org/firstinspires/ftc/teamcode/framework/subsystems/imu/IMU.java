package org.firstinspires.ftc.teamcode.framework.subsystem;

import com.qualcom.hardware.bosch.BNO055IMU;
import com.qualcom.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcom.robotcore.util.ReadWriteFiles;

public class IMU implements IIMU {
    // Creation
    BNO055IMU imu;
    double offset;

    /**
     * Constructor for IMU
     * 
     * @param imu imu sensor
     */
    public IMU(BNO055IMU imu) {
        this.imu = imu;
    }

    @Override
    public double getXAngle() {
        return -imu.getAngularOrientation().thirdAngle - offset;
    }

    @Override
    public double getYAngle() {
        return -imu.getAngularOrientation().secondAngle - offset;
    }

    @Override
    public double getZAngle(){
        return -imu.getAngularOrientation().firstAngle - offset;
    }
    //Gets angle of z-axid with discontinuty
    @Override
    public double getZAngle(double desiredAngle){
        double angle = getZAngle();
        
    }

    @Override
    public double getXAcc(){
        return -imu.getAcceleration().getXAcc;
    }
}