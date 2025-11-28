package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Misc.RobotConstants;

public class IntakeNoSort extends  SubsystemBase{
    DcMotor intakeMotor;
    Servo gate;
    double targetpower,
    gatePosition = RobotConstants.transfer_open_pos;



    public IntakeNoSort(HardwareMap hMap)
    {
        gate = hMap.get(Servo.class, RobotConstants.transfer_servo_name);
        intakeMotor = hMap.get(DcMotor.class, RobotConstants.intake_motor_name);
    }

    public void close()
    {
        gate.setPosition(RobotConstants.transfer_closed_pos);
    }
    public void open()
    {
        gate.setPosition(RobotConstants.transfer_open_pos);
    }
    public void intake()
    {
        targetpower = 1;
    }
    public void stopIntake()
    {
        targetpower = 0;
    }
    public void outtake()
    {
        targetpower = -1;
    }
    @Override
    public void periodic()
    {
        intakeMotor.setPower(targetpower);
        gate.setPosition(gatePosition);
    }
}
