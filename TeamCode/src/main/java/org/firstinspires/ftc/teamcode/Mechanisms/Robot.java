package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public class Robot extends SubsystemBase {
    turretShooter s;
    IntakeNoSort i;
    drive d;

    public Robot(final HardwareMap hmap)
    {
        s = new turretShooter(hmap);
        i = new IntakeNoSort(hmap);
        d = new drive(hmap);
    }

    public void prepShooter()
    {
        double dist = d.yoCalcDist();
        lut.
        s.setSpeed();

    }
}
