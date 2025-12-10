package org.firstinspires.ftc.teamcode.Mechanisms;


import static org.firstinspires.ftc.teamcode.opMode.teleOp.flywheel_speed;
import static org.firstinspires.ftc.teamcode.opMode.teleOp.hood_pos;
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
        dist = sqrt(Math.pow(distanceX, 2) + Math.pow(distanceY, 2));
        unnormalizedHeading = follower.getTotalHeading();
    }


    public Pose getPose() {
        return currentPose;
    }

    public double yoCalcDist() {
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
        /*
        if (dist >= 50.2778 && dist <= 80.2022) {
            return -0.0000167505 * Math.pow(dist, 3)
                    + 0.0030833 * Math.pow(dist, 2)
                    -0.185176 * dist + 4.12003;
        } else {
            return 0;
        }
         */
        return hood_pos;
    }

    public double yoCalcSpeed() {
        /*
        if (dist >= 50.2778 && dist <= 80.2022) {
            return -1700;
        }
        else {
            return 0;
        }
         */
        return flywheel_speed;
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
