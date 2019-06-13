package org.firstinspires.ftc.teamcode.framework.drivetrain;

import org.firstinspires.ftc.teamcode.enums.direction;

public interface IDriveTrain {
    /**
     * Moves the robot in a direction while maintaining a certain orientation
     * 
     * @param currentPosition        The current position of robot in any unit eg.
     *                               encoder counts, sensor distances...
     * @param targetPosition         The target position of the robot in the same
     *                               unit as current position
     * @param rampDownTargetPosition Position at which the robot will start ramping
     *                               down
     * @param rampUpTargetPosition   Position at which the robot will stop ramping
     *                               up in power
     * @param maxPower               The maximum power the robot will move at,
     *                               ranging from 0.0 to 1.0
     * @param lowPower               The lowest power the robot will move at,
     *                               ranging from 0.0 to 1.0
     * @param moveAngle              The angle at which the robot will move in the
     *                               frame of reference of the starting position
     * @param PIDGain                Three gains to control PID feedback loop for
     *                               Orientation correction
     * @param endOrientationAngle    The Direction the robot is facing
     * @return true if the motion is complete, false is the motion is ongoing
     */
    boolean move(double currentPosition, double targetPosition, double rampDownTargetPosition,
            double rampUpTargetPosition, double rampDownEnd, double maxPower, double lowPower, double moveAngle,
            double[] PIDGain, double endOrientationAngle, double allowableDistanceError, double correctiontime);

    boolean pivot();

    void softEncoderReset();

    void resetEncoders();

    double getEncoderDistance();

    void stop();

}