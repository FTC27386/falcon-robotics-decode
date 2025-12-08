package org.firstinspires.ftc.teamcode.Mechanisms;


import static java.lang.Math.sqrt;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Utility.RobotConstants;
import org.firstinspires.ftc.teamcode.Utility.UtilMethods;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;

public class drivetrainSystem extends SubsystemBase {
    public static Pose
            currentPose = new Pose(0, 0, Math.toRadians(90)),
            startPose,
            targ = new Pose(0, 144, 0);
    public Follower follower;
    public double
            x,
            distanceX,
            y,
            distanceY,
            dist,
            heading,
            unnormalizedHeading,
            field_angle,
            zoneBuffer = 7.5 * Math.sqrt(2);
    public Supplier<Pose> poseSupplier = this::getCurrentPose;
    boolean robotCentricDrive = false;

    public drivetrainSystem(HardwareMap hMap) {
        follower = Constants.createFollower(hMap);
        follower.setStartingPose(RobotConstants.autoEndPose == null ? new Pose(8, 8, Math.toRadians(90)) : RobotConstants.autoEndPose);
        follower.update();
    }

    @Override
    public void periodic() {
        follower.update();
        currentPose = follower.getPose();
        x = currentPose.getX();
        y = currentPose.getY();
        heading = currentPose.getHeading();
        distanceX = targ.getX() - x;
        distanceY = targ.getY() - y;
        unnormalizedHeading = follower.getTotalHeading();
    }


    public Pose getPose() {
        return currentPose;
    }

    public double yoCalcDist() //calculate distance in inch
    {
        return sqrt(Math.pow(distanceX, 2) + Math.pow(distanceY, 2));
    }

    public double yoCalcAim()  //calculate adjusted turret angle in degrees
    {
        field_angle = (90 - Math.toDegrees(Math.atan2(distanceY, distanceX)));
        // return Math.toDegrees((heading)-Math.PI/2) + field_angle;
        return -UtilMethods.AngleDifference(Math.toDegrees(unnormalizedHeading), 0) + field_angle;

        //equivalent: Math.toDegrees(currentpose.getHeading() - Math.PI/2)

        //(90 - Math.toDegrees(Math.atan2(distanceY,distanceX)))
    }

    public double yoCalcHood() {
        dist = yoCalcDist();
        if (dist >= 69.674 && dist < 82.260) {
            return 0.000114507 * Math.pow(dist, 2)
                    - 0.0163 * dist
                    + 0.596814;
        } else if (dist >= 82.260 && dist <= 116.262) {
            return -0.00000585763 * Math.pow(dist, 2)
                    + 0.000692307 * dist
                    + 0.0666877;
        } else {
            return 0;
        }
    }

    public double yoCalcSpeed() {
        dist = yoCalcDist();
        if (dist >= 69.674 && dist < 82.260) {
            return 225;
        } else if (dist >= 82.260 && dist <= 116.262) {
            return 270;
        } else {
            return 0;
        }
    }

    public void teleOpDrive(double axial, double lateral, double yaw) {

        follower.setTeleOpDrive(
                -axial,
                -lateral,
                -yaw,
                true);
    }

    public void reloc(Pose reloc) {
        follower.setPose(reloc);
    }

    public Pose getCurrentPose() {
        return currentPose;
    }

    public Pose getTarg() {
        return targ;
    }

    public void relocTarget(Pose reloc) {
        targ = reloc;
    }

    public boolean inZone() {
        return (y > Math.abs(x - 72) + 72 - zoneBuffer) || (y < -Math.abs(x - 72) + 24 + zoneBuffer);
    }


}
