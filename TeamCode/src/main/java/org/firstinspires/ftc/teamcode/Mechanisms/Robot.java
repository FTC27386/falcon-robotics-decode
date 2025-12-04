package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.Utility.RobotConstants;
import org.firstinspires.ftc.teamcode.Utility.closeLUT;
import org.firstinspires.ftc.teamcode.Utility.farLUT;

public class Robot {
    GamepadEx gp;

    shooterSystem s;
    intakeSystem i;
    drivetrainSystem d;
    liftSystem l;
    closeLUT ctable;
    farLUT ftable;
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
        ctable = new closeLUT();
        ftable = new farLUT();
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
    public void prepShooter(RobotConstants.robotState state)
    {
        currentState = state;
        double speed = 0;
        double dist = d.yoCalcDist();
        double hood_pos = 0;
        if(state == RobotConstants.robotState.FAR_SHOOT)
        {
            hood_pos = ftable.lut.get(dist);
            speed = RobotConstants.far_flywheel_speed * RobotConstants.rpm_conversion_factor;
        }
        if(state == RobotConstants.robotState.CLOSE_SHOOT)
        {
            hood_pos = ctable.lut.get(dist);
            speed = RobotConstants.close_flywheel_speed * RobotConstants.rpm_conversion_factor;
        }
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
