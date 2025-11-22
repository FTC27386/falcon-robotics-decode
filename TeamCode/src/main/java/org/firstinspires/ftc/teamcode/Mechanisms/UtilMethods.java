package org.firstinspires.ftc.teamcode.Mechanisms;

public class UtilMethods {
    public static double AngleDifference(double angle1, double angle2) //Taken from Stackoverflow because I am lazy
    {
        double diff = (angle2 - angle1 + 180) % 360 - 180;
        return diff < -180 ? diff + 360 : diff;
    }
}
