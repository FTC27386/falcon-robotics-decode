package org.firstinspires.ftc.teamcode.Utility;

import com.seattlesolvers.solverslib.util.InterpLUT;

public class closeLUT
{

    double[][] hoodData =
            {
                    {60.57, 0},
                    {72.76,0.01},
                    {92.92,0.04},
                    {200,0.045}
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
