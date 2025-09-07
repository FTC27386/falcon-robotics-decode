
package org.firstinspires.ftc.teamcode.Misc;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class RobotConstants {
    public static int armVerticalPos = -478;
    public static double armP = 0.003;
    public static double armI = 0;
    public static double armD = 0.0003;
    public static double armkG = .05;
    public static double armL = .03;
    public static double armHeight = 8.5;
    public static double ticks_in_deg = 1725.1 / 360;
    public static double armGearRatio = 3;
    public static double ServoExtension = 8.000;
    public static double armBaseLength = 14.75;
    public static int armDefaultBottom = -1825;
    public static double armMeasuredBottomAngle = 59.155;
    public static double clawOpen = .24;
    public static double clawClosed = .01;
    //Lift
    public static int liftSpecimen = 2700;
    public static double liftP = 0.003;
    public static double liftI = 0;
    public static double liftD = 0.00005;
    public static double liftkG = 0.06;
    //drive
    public static double lockP = .007;
    public static double lockI = 0;
    public static double lockD = 0;
    public static double bridgeUpPos = .15;
    public static double bridgeDownPos = .85;
}
