package org.firstinspires.ftc.teamcode.Misc;

import com.seattlesolvers.solverslib.util.InterpLUT;

public class closeLUT
{

    double[][] hoodData =
            {
                    {0, 0},
                    {1,1},
                    {2,2},
                    {300,3}
            };

    public InterpLUT lut;
    public void addPoint(double in, double out)
    {
        lut.add(in,out);
    }
    public closeLUT()
    {
        lut = new InterpLUT();
        for (double[] point : hoodData) {
            addPoint(point[0], point[1]);
        }
        lut.createLUT();
    }

}
