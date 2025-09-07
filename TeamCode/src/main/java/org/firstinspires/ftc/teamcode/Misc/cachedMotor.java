package org.firstinspires.ftc.teamcode.Misc;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class cachedMotor {
    public DcMotorEx thisMotor;
    double cachingTol;
    double currPower;
    public cachedMotor(DcMotorEx motor, double cachingTol )
    {
        thisMotor = motor;
        this.cachingTol = cachingTol;
    }
    public void setPower(double newPower)
    {
        if (Math.abs(currPower-newPower) > cachingTol || newPower == 0)
        {
            currPower = newPower;
        }
    }
    public int getCurrentPosition()
    {
        return thisMotor.getCurrentPosition();
    }
    public void setCachingTol(double cachingTol)
    {
        this.cachingTol = cachingTol;
    }
}
