
package org.firstinspires.ftc.teamcode.Misc;

public class UtilMethods {
    public static double squareRootMagnitude(double num)
    {
        return Math.signum(num) * Math.sqrt(Math.abs(num));
    }
    public static double AngleDifference(double angle1, double angle2) //Taken from Stackoverflow because I am lazy
    {
        double diff = (angle2 - angle1 + 180) % 360 - 180;
        return diff < -180 ? diff + 360 : diff;
    }
    public static double squareMagnitude(double input)
    {
        double sign = input < 0? -1 : 1;
        return Math.pow(input, 2) * sign;
    }

}
