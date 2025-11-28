package org.firstinspires.ftc.teamcode.Misc;

import com.seattlesolvers.solverslib.util.InterpLUT;

public class farLUT
{
    double[][] hoodData =
            {
                    {0.0, 0.0},
                    {1.0,1.0},
                    {2.0,2},
                    {300.3,2}
            };
    public InterpLUT lut;
    public void addPoint(double in, double out)
    {
        lut.add(in,out);
    }
    public farLUT()
    {
        lut = new InterpLUT();
        for (double[] point : hoodData) {
            this.addPoint(point[0], point[1]);
        }
        lut.createLUT();
    }


}
