package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorGroup;

import org.firstinspires.ftc.teamcode.Misc.cachedMotor;

public class Shooter
{
    Motor shooter1;
    Motor shooter2;
    MotorGroup shooter = new MotorGroup(shooter1, shooter2);
}