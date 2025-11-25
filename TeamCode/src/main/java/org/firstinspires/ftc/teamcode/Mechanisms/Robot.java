package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.Misc.RobotConstants;
import org.firstinspires.ftc.teamcode.Misc.closeLUT;
import org.firstinspires.ftc.teamcode.Misc.farLUT;

import java.sql.Driver;
import java.util.function.DoubleSupplier;

public class Robot {
    GamepadEx gp;

    turretShooter s;
    IntakeNoSort i;
    drive d;
    closeLUT ctable;
    farLUT ftable;
    RobotConstants.robotState currentState;

    public turretShooter getS() {
        return s;
    }

    public IntakeNoSort getI() {

        return i;
    }

    public drive getD()
    {
        return d;
    }

    public Robot(final HardwareMap hmap)
    {
        s = new turretShooter(hmap);
        i = new IntakeNoSort(hmap);
        d = new drive(hmap);
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
            speed = 2;
        }
        if(state == RobotConstants.robotState.CLOSE_SHOOT)
        {
            hood_pos = ctable.lut.get(dist);
            speed = 1;
        }
        double ang = d.yoCalcAim();
        s.setSpeed(speed);
        s.setHoodPosition(hood_pos);
        s.setTurretPosition(ang);
    }
}
