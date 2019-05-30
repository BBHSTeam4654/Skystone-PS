package org.firstinspires.ftc.teamcode.framework.drivetrain;

import org.firstinspires.ftc.teamcode.enums.direction;

public interface IDriveTrain{
    /**
     * 
     */
    boolean move();

    boolean pivot();
    
    void softEncoderReset();

    void resetEncoders();

    double getEncoderDistance();

    void stop();

}