package org.firstinspires.ftc.teamcode.Mechanisms;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Misc.RobotConstants;
import org.firstinspires.ftc.teamcode.pedroCalibration.Constants;

public class drive extends SubsystemBase {
private Follower follower;
public static Pose startPose;
boolean robotCentricDrive = false;

public drive(HardwareMap hMap)
{
    follower = Constants.createFollower(hMap);
    follower.setStartingPose(RobotConstants.autoLastPose == null ? new Pose() : startPose);
}
public void periodic()
{
    follower.update();
}

public void teleOpDrive(double axial, double lateral, double rot)
{
    follower.setTeleOpDrive(
            -axial,
            -lateral,
            -rot,
            true);
}

public Pose getCurrentPose()
{
    return follower.getPose();
}




}
