package org.firstinspires.ftc.teamcode.framework;

public class Utility {
    public static double roundTwoDec(double x) {
        x *= 100;
        x = Math.round(x);
        return x / 100;
    }
}