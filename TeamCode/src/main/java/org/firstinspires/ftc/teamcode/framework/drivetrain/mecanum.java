package org.firstinspires.ftc.teamcode.framework.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.framework.drivetrain.IDriveTrain;
import org.firstinspires.ftc.teamcode.enums.Direction;
import org.firstinspires.ftc.teamcode.framework.subsystems.imu.IIMU;
import org.firstinspires.ftc.teamcode.framework.Datalog;
import org.firstinspires.ftc.teamcode.framework.Utility;

import java.util.List;
public class Mecanum implements IDriveTrain {
    private List<DcMotor> motors;
    private List<DcMotor> encoders;
    private IIMU imu;

    private Datalog data;

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
     * 
     * @param motors   List of motors on drivetrain in order of Right Front, Right
     *                 Back, Left Front and then Left Back
     * @param imu      the inertial measurement unit or gyro sensor of the robot
     * @param encoders List of encoders (passed in as DcMotor) on drivetrain to
     *                 calculate distances and positions
     */
    public void MecanumDrive(List<DcMotor> motors, IIMU imu, Telemetry telemetry, List<DcMotor> encoders) {
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
        motors.get(0).setPower(rfPower);
        motors.get(1).setPower(rbPower);
        motors.get(2).setPower(lfPower);
        motors.get(3).setPower(lbPower);
    }

    // translation of vertical, horizontal, pivot power into motor speeds
    public void rawSlide(double horizontal, double vertical, double pivot, double maxPower) {
        double powers[] = { vertical - horizontal + pivot, vertical + horizontal + pivot, vertical + horizontal - pivot,
                vertical - horizontal - pivot };

        if (horizontal != 0 || vertical != 0) {
            int max = 0;
            int counter = 0;

            for (double element : powers) {
                if (Math.abs(element) > Math.abs(powers[max])) {
                    max = counter;
                }
                counter++;
            }

            double maxCalculatedPower = Math.abs(powers[max]);

            if (maxCalculatedPower != 0) {
                powers[0] = powers[0] / maxCalculatedPower * maxPower;
                powers[1] = powers[1] / maxCalculatedPower * maxPower;
                powers[2] = powers[2] / maxCalculatedPower * maxPower;
                powers[3] = powers[3] / maxCalculatedPower * maxPower;

            }
            this.setPowerAll(powers[0], powers[1], powers[2], powers[3]);
        }
    }
    // calculate power for x direction

    private double calculateX(double desiredAngle, double speed) {
        return Math.sin(Math.toRadians(desiredAngle)) * speed;
    }

    // calculate power for y direction
    private double calculateY(double desiredAngle, double speed) {
        return Math.cos(Math.toRadians(desiredAngle)) * speed;
    }

    @Override
    public void resetEncoders() {
        for (DcMotor motor : encoders) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        leftVerticalLastEncoder = 0;
        rightVerticalLastEncoder = 0;
        horizontalLastEncoder = 0;
    }

    public void resetEncoders(DcMotor.RunMode endMode) {
        for (DcMotor motor : encoders) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(endMode);
        }
        leftVerticalLastEncoder = 0;
        rightVerticalLastEncoder = 0;
        horizontalLastEncoder = 0;
    }

    public boolean move(double currentPosition, double targetPosition, double rampDownTargetPosition,
            double rampUpTargetPosition, double rampDownEnd, double maxPower, double lowPower, double moveAngle,
            double[] PIDGain, double endOrientationAngle, double allowableDistanceError, double correctionTime) {
        double positionDifference = targetPosition - currentPosition;

        if (Math.abs(positionDifference) <= allowableDistanceError) {
            this.stop();

            if (!targetReached) {
                targetReached = true;
                distanceCorrectionTimer.reset();
                return true;
            }
        } else {
            double rampDownDifference = targetPosition - rampDownTargetPosition;
            double rampDownEndDifference = targetPosition - rampDownEnd;

            double power;
            if (rampDownDifference >= Math.abs(positionDifference)) {
                power = lowPower;
            } else if (rampDownDifference > Math.abs(positionDifference)) {
                power = Math.abs((positionDifference) - rampDownDifference)
                        * ((maxPower - lowPower) / (rampDownDifference - rampDownEndDifference)) + maxPower;
            } else {
                power = maxPower;
            }

            // get current IMU angle **MAY NEED TO ALTER ANGLE BASED ON HUB ORIENTATION**
            double currentAngle = imu.getZAngle(endOrientationAngle);

            moveAngle = moveAngle - currentAngle;

            if (moveAngle <= -180) {
                moveAngle += 360;
            }
            if (positionDifference < 0) {
                moveAngle += 180;
            }

            // x vector movement
            double horizontal = Utility.roundTwoDec(calculateX(moveAngle, power));
            // y vector movement
            double vertical = Utility.roundTwoDec(calculateY(moveAngle, power));
            // correction
            double pivotCorrection = ((currentAngle - endOrientationAngle) * PIDGain[0]);

            rawSlide(horizontal, vertical, pivotCorrection, power);
        }
        if(targetReached&&distanceCorrectionTimer.milliseconds()>=correctionTime){
            this.stop();
            targetReached=false;
            return false;
        }else{
            return false;
        }
    }

    @Override 
    public void stop(){
        setPowerAll(0, 0, 0, 0);
    }
}
