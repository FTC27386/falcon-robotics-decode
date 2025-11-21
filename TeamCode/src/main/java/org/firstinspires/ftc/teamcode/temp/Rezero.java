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
package org.firstinspires.ftc.teamcode.temp;

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
@TeleOp(name = "Game", group = "Robot")

public class Rezero extends OpMode {
    public static double flywheelPower = 0;
    public static double increment = 0.000;
    public static int swayinterval = 1500;
    ElapsedTime swayTimer = new ElapsedTime();
    int servodirection = 1;
    public static int servoValue = 0;
    // Servo turretServo;
    DcMotor[] flywheel = new DcMotor[2];
    Servo[] turretServo = new Servo[2];
    Servo hood;
    double FLYWHEEL_SPEED;
    double MIN_FLYWHEEL_SPEED;
    double MAX_FLYWHEEL_SPEED;
    double HOOD_ANGLE;
    double MIN_HOOD_ANGLE;
    double MAX_HOOD_ANGLE;
    public static double TURRET_ANGLE;
    double MIN_TURRET_ANGLE;
    double MAX_TURRET_ANGLE;

    @Override
    public void init() {
        swayTimer.reset();
        FLYWHEEL_SPEED = 0;
        MIN_FLYWHEEL_SPEED = 0;
        MAX_FLYWHEEL_SPEED = 1;

        HOOD_ANGLE = 0.6;
        MIN_HOOD_ANGLE = 0.6;
        MAX_HOOD_ANGLE = 0.7;

        TURRET_ANGLE = 0.65;
        MIN_TURRET_ANGLE = 0.0;
        MAX_TURRET_ANGLE = 1;

        flywheel[0] = hardwareMap.get(DcMotor.class, "flywheel1");
        flywheel[1] = hardwareMap.get(DcMotor.class, "flywheel2");

        turretServo[0] = hardwareMap.get(Servo.class, "turretServo1");
        turretServo[1] = hardwareMap.get(Servo.class, "turretServo2");

       // hood = hardwareMap.get(Servo.class, "hood");

        for (DcMotor dcMotor : flywheel) {
            dcMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            dcMotor.setZeroPowerBehavior(FLOAT);
        }
    }

    @Override
    public void loop() {
        if(swayTimer.milliseconds() > swayinterval)
        {
            swayTimer.reset();
            servodirection *= -1;
        }
        telemetry.addLine("Right trigger turns flywheel on");
        telemetry.addLine("Right stick controls hood and turret angle");

        HOOD_ANGLE -= 0.001 * gamepad1.right_stick_y;
        FLYWHEEL_SPEED = (gamepad1.right_trigger > 0 ? 1 : 0);

        // Clamp both values between MIN and MAX.
        FLYWHEEL_SPEED = clamp(FLYWHEEL_SPEED, MIN_FLYWHEEL_SPEED, MAX_FLYWHEEL_SPEED);
        TURRET_ANGLE = clamp(TURRET_ANGLE, MIN_TURRET_ANGLE, MAX_TURRET_ANGLE);
        HOOD_ANGLE = clamp(HOOD_ANGLE, MIN_HOOD_ANGLE, MAX_HOOD_ANGLE);

        // Send values to turret
        turret(FLYWHEEL_SPEED, HOOD_ANGLE, TURRET_ANGLE);
    }
    private void turret(double speed, double hoodAngle, double turretAngle) {
        for (DcMotor dcMotor : flywheel) dcMotor.setPower(flywheelPower);
        for (Servo servo : turretServo) servo.setPosition(turretAngle);
        //hood.setPosition(hoodAngle);
        telemetry.addData("Flywheel Motor 1", "Power %5.2f", flywheel[0].getPower());
        telemetry.addData("Flywheel Motor 2", "Power %5.2f", flywheel[1].getPower());
        //telemetry.addData("Hood", "Angle %5.3f", hood.getPosition());
        telemetry.addData("Turret Motor 1", "Angle %5.3f", turretServo[0].getPosition());
        telemetry.addData("Turret Motor 2", "Angle %5.3f", turretServo[1].getPosition());
    }
}
