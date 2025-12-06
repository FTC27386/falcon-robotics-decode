package org.firstinspires.ftc.teamcode.Misc;

import com.seattlesolvers.solverslib.util.InterpLUT;

public class testLUT
{

    double[][] hoodData =
            {
                    {0, 0},
                    {1,1},
                    {2,2},
                    {300,3}
            };

    public InterpLUT lut;
    public testLUT()
    {
        lut = new InterpLUT();
        for (double[] hoodDatum : hoodData) {
            lut.add(hoodDatum[0], hoodDatum[1]);
        }
        lut.createLUT();
    }

}
