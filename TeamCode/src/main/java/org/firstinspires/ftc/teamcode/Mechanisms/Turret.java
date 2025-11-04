/* Copyright (c) 2025 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.firstinspires.ftc.teamcode.Mechanisms;

import static androidx.core.math.MathUtils.clamp;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This OpMode illustrates how to program your robot to drive field relative.  This means
 * that the robot drives the direction you push the joystick regardless of the current orientation
 * of the robot.
 *
 * This OpMode assumes that you have four mecanum wheels each on its own motor named:
 *   front_left_motor, front_right_motor, back_left_motor, back_right_motor
 *
 *   and that the left motors are flipped such that when they turn clockwise the wheel moves backwards
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 *
 */
@Config
@TeleOp(name = "Turret", group = "Robot")

public class Turret extends OpMode {
    public static double increment = 0.001;
    public static int swayinterval = 1500;
    ElapsedTime swayTimer;
    int servodirection = 1;
    public static int servoValue = 0;
    Servo turretServo;
    DcMotor flywheel1;
    double turret_angle;
    DcMotor flywheel2;
    Servo hood;
    double FLYWHEEL_SPEED;
    double MAX_FLYWHEEL_SPEED;
    double MIN_FLYWHEEL_SPEED;
    double HOOD_ANGLE;
    double MAX_HOOD_ANGLE;
    double MIN_HOOD_ANGLE;

    @Override
    public void init() {

        swayTimer = new ElapsedTime();
        servodirection = 0;
        FLYWHEEL_SPEED = 1;
        HOOD_ANGLE = 0.5;
        MAX_FLYWHEEL_SPEED = 1;
        MIN_FLYWHEEL_SPEED = 0;
        MAX_HOOD_ANGLE = 1;
        MIN_HOOD_ANGLE = 0;
        turret_angle = 0;

        turretServo = hardwareMap.get(Servo.class, "turretServo");

        flywheel1 = hardwareMap.get(DcMotor.class, "flywheel1");
        flywheel2 = hardwareMap.get(DcMotor.class, "flywheel2");
        hood = hardwareMap.get(Servo.class, "hood");

        flywheel1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel1.setZeroPowerBehavior(FLOAT); //Makes the flywheel1 not turn itself off
        flywheel2.setZeroPowerBehavior(FLOAT);
    }

    @Override
    public void loop() {
        if (swayTimer.milliseconds() > swayinterval)
        {
            servodirection = -1 * servodirection;
            swayTimer.reset();
        }

        servoValue += increment * servodirection;
        telemetry.addLine("Press A to reset Yaw");
        telemetry.addLine("The left joystick sets the robot direction");
        telemetry.addLine("Moving the right joystick left and right turns the robot");
        telemetry.addLine("Right trigger runs intake forward, right bumper runs intake backward");
        telemetry.addLine("Left trigger turns flywheel1 on");
        telemetry.addLine("Dpad up increases flywheel1 speed, dpad down decreases flywheel1 speed");
        telemetry.addLine("Dpad left/right controls hood angle");
        telemetry.addLine("Using dpad while holding down left bumper allows finer control");

        if(gamepad1.dpadUpWasPressed()) {
            if (gamepad1.left_bumper) {
                FLYWHEEL_SPEED += 0.01;
            }
            else {
                FLYWHEEL_SPEED += 0.1;
            }
        }
        if(gamepad1.dpadDownWasPressed()) {
            if (gamepad1.left_bumper) {
                FLYWHEEL_SPEED -= 0.01;
            }
            else {
                FLYWHEEL_SPEED -= 0.1;
            }
        }
        if(gamepad1.dpadRightWasPressed()) {
            if (gamepad1.left_bumper) {
                HOOD_ANGLE += 0.01;
            }
            else {
                HOOD_ANGLE += 0.1;
            }
        }
        if(gamepad1.dpadLeftWasPressed()) {
            if (gamepad1.left_bumper) {
                HOOD_ANGLE -= 0.01;
            }
            else {
                HOOD_ANGLE -= 0.1;
            }
        }
        if(gamepad1.triangleWasPressed())
        {
            turret_angle += 0.05;
        }
        if(gamepad1.crossWasPressed())
        {
            turret_angle -= 0.05;
        }
        // Clamp both values between MIN and MAX.
        FLYWHEEL_SPEED = clamp(FLYWHEEL_SPEED, MIN_FLYWHEEL_SPEED, MAX_FLYWHEEL_SPEED);
        HOOD_ANGLE = clamp(HOOD_ANGLE, MIN_HOOD_ANGLE, MAX_HOOD_ANGLE);
        turret_angle = clamp(turret_angle, 0, 1);
        servoValue = clamp(servoValue, 0, 1);
        if(gamepad1.left_trigger > 0) turret(FLYWHEEL_SPEED, HOOD_ANGLE, servoValue);
        else turret(0, HOOD_ANGLE, servoValue);
    }

    private void turret(double speed, double angle, double turretAngle) {
        flywheel1.setPower(speed);
        flywheel2.setPower(speed);
        hood.setPosition(angle);
        turretServo.setPosition(turretAngle);
        telemetry.addData("Flywheel Power", "Speed %5.2f", flywheel1.getPower());
        telemetry.addData("Hood Value", "Angle %5.2f", hood.getPosition());
    }
}
