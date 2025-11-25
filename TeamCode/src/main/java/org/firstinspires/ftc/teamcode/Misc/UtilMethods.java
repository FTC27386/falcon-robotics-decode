
package org.firstinspires.ftc.teamcode.Misc;

public class UtilMethods {
    public static double squareRootMagnitude(double num)
    {
        return Math.signum(num) * Math.sqrt(Math.abs(num));
    }
    public static double squareMagnitude(double input)
    {
        double sign = input < 0? -1 : 1;
        return Math.pow(input, 2) * sign;
    }

}
