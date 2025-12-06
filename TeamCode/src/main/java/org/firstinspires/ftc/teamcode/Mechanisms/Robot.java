package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.Utility.RobotConstants;

public class Robot {
    GamepadEx gp;

    shooterSystem s;
    intakeSystem i;
    drivetrainSystem d;
    liftSystem l;
    RobotConstants.robotState currentState;

    public shooterSystem getS() {
        return s;
    }

    public intakeSystem getI() {
        return i;
    }

    public drivetrainSystem getD()
    {
        return d;
    }

    public Robot(final HardwareMap hmap)
    {
        s = new shooterSystem(hmap);
        i = new intakeSystem(hmap);
        d = new drivetrainSystem(hmap);
        l = new liftSystem(hmap);
        currentState = RobotConstants.robotState.IDLE;
        for (LynxModule mod : hmap.getAll(LynxModule.class))
        {
            mod.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }
    public void periodic()
    {
        i.periodic();
        s.periodic();
        d.periodic();
        l.periodic();
    }
    public void setShooterValues()
    {
        double speed = 0;
        double dist = d.yoCalcDist();
        double hood_pos = 0;

        /*
        if (dist >= 69.674 && dist < 82.260) {
            hood_pos = 0.000114507 * Math.pow(dist, 2)
                    - 0.0163 * dist
                    + 0.596814;
            speed = 225;
        } else if (dist >= 82.260 && dist <= 116.262) {
            hood_pos = -0.00000585763 * Math.pow(dist, 2)
                    + 0.000692307 * dist
                    + 0.0666877;
            speed = 270;
        } else {
            speed = 0;
        }
        speed = 0;
         */
        speed = 1500;

        double ang = d.yoCalcAim();
        s.setSpeed(speed);
        s.setHoodPosition(hood_pos);
        s.setTurretPosition(ang);
    }
    public void climb()
    {
        l.down();
        l.unlatch();
    }

}
