package org.firstinspires.ftc.teamcode.framework.subsystems.imu;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.util.ReadWriteFile;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import java.io.File;

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
    public double getZAngle() {
        return -imu.getAngularOrientation().firstAngle - offset;
    }

    // Gets angle of z-axid with discontinuty
    @Override
    public double getZAngle(double desiredAngle) {
        double angle = getZAngle();
        if (angle < desiredAngle - 180) {
            angle += 360;
        } else if (angle > desiredAngle + 180) {
            angle -= 360;
        }
        return angle;
    }

    @Override
    public double getXAcc() {
        return imu.getAcceleration().xAccel;
    }

    @Override
    public double getYAcc() {
        return imu.getAcceleration().yAccel;
    }

    @Override
    public double getZAcc() {
        return imu.getAcceleration().zAccel;
    }

    @Override
    public double getXVelo() {
        return imu.getVelocity().xVeloc;
    }

    @Override
    public double getYVelo() {
        return imu.getVelocity().yVeloc;
    }

    @Override
    public double getZVelo() {
        return imu.getVelocity().zVeloc;
    }

    @Override
    public void initialize() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);
    }

    @Override
    public void setOffSet(double offset) {
        this.offset = offset;
    }

    @Override
    public void setAsZero() {
        offset = -imu.getAngularOrientation().firstAngle;
    }
}