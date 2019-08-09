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
        if (targetReached && distanceCorrectionTimer.milliseconds() >= correctionTime) {
            this.stop();
            targetReached = false;
            return false;
        } else {
            return true;
        }
    }

    /**
     * Pivots the robot to a desired angle, while using a proportional control loop
     * to maintain the robot's drive speed
     * 
     * @param desiredAngle         The angle to which to pivot to
     * @param rampDownAngle        The angle at which to start slowing down
     * @param maxPower             The max power to pivot at, ranging from 0.0 to
     *                             1.0
     * @param minPower             The min power to pivot at, ranging from 0.0 to
     *                             1.0
     * @param correctionAngleError
     * @param correctionTime       The amount of time to spend correcting to stay
     *                             within the desired range
     * @param direction
     * @return true if the action has been completed, false if the robot is still
     *         pivoting
     */
    @Override
    public boolean pivot(double desiredAngle, double rampDownAngle, double maxPower, double minPower,
            double correctionTime, double correctionAngleError, Direction direction) {
        double currentAngle = imu.getZAngle(desiredAngle);
        double angleDifference = desiredAngle - currentAngle;
        double rampDownDifference = desiredAngle - rampDownAngle;
        double power;

        // calculate power
        if (Math.abs(angleDifference) > Math.abs(rampDownDifference)) {
            power = maxPower;
        } else {
            power = (maxPower - minPower) / (Math.abs(rampDownDifference)) * Math.abs(angleDifference) + minPower;
        }
        // turn clockwise or counterclockwise depending on which side of desired angle
        // current angle is
        if (direction == Direction.FASTEST || targetReached) {
            if (angleDifference > 0) {
                this.setPowerAll(-power, -power, power, power);
            } else {
                this.setPowerAll(power, power, -power, -power);
            }
        } else if (direction == Direction.CLOCKWISE) {
            this.setPowerAll(-power, -power, power, power);
        } else {
            this.setPowerAll(power, power, -power, -power);
        }

        // determine if the pivoting angle is in the desired range
        if (Math.abs(angleDifference) < correctionAngleError && !targetReached) {
            pivotTime.reset();
            targetReached = true;
        }
        if (targetReached && pivotTime.milliseconds() >= correctionTime) {
            targetReached = false;
            this.stop();
            return false;
        } else {
            return true;
        }
    }

    @Override
    public void softEncoderReset() {
        leftVerticalLastEncoder = encoders.get(0).getCurrentPosition();
        rightVerticalLastEncoder = encoders.get(1).getCurrentPosition();
        horizontalLastEncoder = encoders.get(2).getCurrentPosition();
    }

    private double[] getEncoderPositions() {
        double[] encoders = { this.encoders.get(0).getCurrentPosition() - leftVerticalLastEncoder,
                this.encoders.get(1).getCurrentPosition() - rightVerticalLastEncoder,
                this.encoders.get(2).getCurrentPosition() - horizontalLastEncoder };
        return encoders;
    }

    public double getEncoderDistance() {

        // Get Current Positions
        double[] encoders = this.getEncoderPositions();

        double vlPos = encoders[0];
        double vrPos = encoders[1];
        double hPos = encoders[2];

        // Average the Vertical Wheels
        double y = ((Math.abs(vlPos) + Math.abs(vrPos)) / 2);
        double x = hPos;

        // Calculate distance
        double distance = Math.sqrt(Math.pow(y, 2) + Math.pow(x, 2));

        return distance;
    }

    @Override
    public void stop() {
        setPowerAll(0, 0, 0, 0);
    }
}
