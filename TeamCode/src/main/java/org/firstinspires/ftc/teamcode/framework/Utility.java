package org.firstinspires.ftc.teamcode.framework;

public class Utility {
    public static double roundTwoDec(double x) {
        x *= 100;
        x = math.round(x);
        return x / 100;
    }
}