package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDController;

import org.firstinspires.ftc.teamcode.Utility.RobotConstants;

public class liftSystem extends SubsystemBase {

    DcMotor lift_motor;
    Servo latch_left, latch_right;
    PIDController motor_controller;
    double power;
    int currentPos;
    boolean activated;

    public liftSystem(HardwareMap hmap)
    {
        motor_controller = new PIDController(RobotConstants.lift_kP, 0, RobotConstants.lift_kD);
        motor_controller.setSetPoint(0);
        lift_motor = hmap.get(DcMotor.class, RobotConstants.lift_motor_name);
        latch_left = hmap.get(Servo.class, RobotConstants.left_lift_servo_name);
        latch_right = hmap.get(Servo.class,RobotConstants.right_lift_servo_name);
    }
    @Override
    public void periodic() {
        if(activated)
        {
            currentPos = lift_motor.getCurrentPosition();
            power = activated?
                    motor_controller.calculate(currentPos, RobotConstants.top_climb_position): 0;
            lift_motor.setPower(power);
        }
    }
    public void unlatch()
    {
        latch_left.setPosition(RobotConstants.latch_open_pos);
        latch_right.setPosition(RobotConstants.latch_open_pos);
    }
    public void latch()
    {
        latch_right.setPosition(RobotConstants.latch_close_pos);
        latch_left.setPosition(RobotConstants.latch_close_pos);
    }
    public void down()
    {
        motor_controller.setSetPoint(RobotConstants.top_climb_position);
    }




}
