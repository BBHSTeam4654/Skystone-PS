package org.firstinspires.ftc.teamcode.framework.drivetrain;

import com.qualcom.robotcore.hardware.DcMotor;
import com.qualcom.robotcore.util.ElapsedTime;
import com.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.drivetrain.IDriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.imu.IIMU;
import org.firstinspires.ftc.teamcode.enums.Direction;
import org.firstinspires.ftc.teamcode.framework.Datalog;

public class Mecanum implements IDriveTrain {
    private List<DcMotor> motors;
    private List<DcMotor> encoders;
    private IIMU imu;

    private DataLog data;

    private final double END_ANGLE_OFFSET = 5;

    private ElapsedTime pivotTime;
    private ElapsedTime distanceCorrectionTimer;

    private boolean targetReached = false;
    private boolean needsToPivot = false;

    private double leftVerticalLastEncoder = 0;
    private double rightVerticalLastEncoder = 0;
    private double horizontalLastEncoder = 0;

    Telemetry telemetry;

    /**
     * Constructor for mecanum drivetrain with free-spinning odometry wheels
     * @param motors List of motors on drivetrain in order of Right Front, Right Back, Left Front and then Left Back
     * @param imu the inertial measurement unit or gyro sensor of the robot
     * @param encoders List of encoders (passed in as DcMotor) on drivetrain to calculate distances and positions
     */
    public MecanumDrive(List<DcMotor> motors, IIMU imu, Telemetry telemetry, List<DcMotor> encoders){
        this.motors = motors;
        this.imu = imu;
        this.imu.initialize();
        this.telemetry = telemetry;
        this.encoders = encoders;
        pivotTime = new ElapsedTime();
        distanceCorrectionTimer = new ElapsedTime();
    }

    /**
    * @param rfPower right front power
    * @param rbPower right back power
    * @param lfPower left front power
    * @param lbPower left back power
    */
    private void setPowerAll(double rfPower, double rbPower, double lfPower, double lbPower) {
        motors.get(0).setpower(rfPower);
        motors.get(1).setpower(rbPower);
        motors.get(2).setpower(lfPower);
        motors.get(3).setpower(lbPower);
    }

    

}
