package org.firstinspires.ftc.teamcode.Mechanisms;


import static java.lang.Math.sqrt;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.command.button.Button;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.Misc.RobotConstants;
import org.firstinspires.ftc.teamcode.pedroCalibration.Constants;

import java.util.function.Supplier;

public class drive extends SubsystemBase {
private Follower follower;
public static Pose
        currentPose,
        startPose,
    targ;
boolean robotCentricDrive = false;
public double
        x,
        distanceX,
        y,
        distanceY,
    heading;

public Supplier<Pose> poseSupplier = this::getCurrentPose;


public drive(HardwareMap hMap)
{
    follower = Constants.createFollower(hMap);
    follower.setStartingPose(RobotConstants.autoEndPose == null ? new Pose(0,0,0) : RobotConstants.autoEndPose);

}
    @Override
public void periodic(){
    follower.update();
    currentPose = follower.getPose();
    x = currentPose.getX();
    y = currentPose.getY();
    heading = currentPose.getHeading();
    distanceX = x - targ.getX();
    distanceY = y - targ.getY();
}


public Pose getPose()
{
    return currentPose;
}

public double yoCalcDist() //calculate distance in inch
{
     return sqrt(Math.pow(distanceX, 2) + Math.pow(distanceY, 2));
}

public double yoCalcAim()  //calculate adjusted turret angle in rad
{
    double field_angle = (90 - Math.toDegrees(Math.atan2(distanceY,distanceX)));
    return -heading - field_angle;
}

public void teleOpDrive(GamepadEx gamepad)
{
    follower.setTeleOpDrive(
            -gamepad.getLeftY(),
            -gamepad.getLeftX(),
            -gamepad.getRightX(),
            true);
}
public void reloc(Pose reloc)
{
    follower.setPose(reloc);
}
public Pose getCurrentPose()
{
    return currentPose;
}




}
