package org.firstinspires.ftc.teamcode.Misc;

import com.seattlesolvers.solverslib.util.InterpLUT;

public class shooterInterPLUT
{

    InterpLUT lut;
    public void addPoint()
    {
        lut.add(0,0);
    }
    public shooterInterPLUT()
    {
        lut = new InterpLUT();
    }


}
