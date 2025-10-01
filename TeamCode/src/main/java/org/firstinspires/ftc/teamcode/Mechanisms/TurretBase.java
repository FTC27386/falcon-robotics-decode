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

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

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
@TeleOp(name = "Robot: Field Relative Mecanum Drive", group = "Robot")

public class TurretBase extends OpMode {
    // This declares the four motors needed
    DcMotor intake;
    DcMotor flywheel;
    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;
    Servo hood;

    // This declares the IMU needed to get the current direction the robot is facing
    IMU imu;

    double flywheelSpeed;
    double hoodAngle;
    double MAX_FLYWHEEL_SPEED;
    double MIN_FLYWHEEL_SPEED;
    double MAX_ANGLE;
    double MIN_ANGLE;

    @Override
    public void init() {
        flywheelSpeed = 1;
        hoodAngle = 0.5;
        MAX_FLYWHEEL_SPEED = 1;
        MIN_FLYWHEEL_SPEED = 0;
        MAX_ANGLE = 1;
        MIN_ANGLE = 0;

        intake = hardwareMap.get(DcMotor.class, "intake");
        flywheel = hardwareMap.get(DcMotor.class, "flywheel");
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");

        hood = hardwareMap.get(Servo.class, "hood");

        // We set the left motors in reverse which is needed for drive trains where the left
        // motors are opposite to the right ones.
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        // This uses RUN_USING_ENCODER to be more accurate.   If you don't have the encoder
        // wires, you should remove these
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setDirection(DcMotor.Direction.REVERSE);
        flywheel.setZeroPowerBehavior(FLOAT); //Makes the flywheel not turn itself off

        imu = hardwareMap.get(IMU.class, "imu");
        // This needs to be changed to match the orientation on your robot
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        RevHubOrientationOnRobot orientationOnRobot = new
                RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

    @Override
    public void loop() {
        telemetry.addLine("Press A to reset Yaw");
        telemetry.addLine("The left joystick sets the robot direction");
        telemetry.addLine("Moving the right joystick left and right turns the robot");
        telemetry.addLine("Right trigger runs intake forward, right bumper runs intake backward");
        telemetry.addLine("Left trigger turns flywheel on");
        telemetry.addLine("Dpad up increases flywheel speed, dpad down decreases flywheel speed");
        telemetry.addLine("Dpad left/right controls hood angle");
        telemetry.addLine("Using dpad while holding down left bumper allows finer control");

        // If you press the A button, then you reset the Yaw to be zero from the way
        // the robot is currently pointing
        if (gamepad1.a) {
            imu.resetYaw();
        }
        driveFieldRelative(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        if(gamepad1.right_trigger > 0) {
            intake.setPower(1);
        }
        else if (gamepad1.right_bumper) {
            intake.setPower(-1);
        }
        else {
            intake.setPower(0);
        }

        if(gamepad1.dpadUpWasPressed()) {
            if (gamepad1.left_bumper) {
                flywheelSpeed += 0.01;
            }
            else {
                flywheelSpeed += 0.1;
            }
        }
        if(gamepad1.dpadDownWasPressed()) {
            if (gamepad1.left_bumper) {
                flywheelSpeed -= 0.01;
            }
            else {
                flywheelSpeed -= 0.1;
            }
        }
        if(gamepad1.dpadRightWasPressed()) {
            if (gamepad1.left_bumper) {
                hoodAngle += 0.01;
            }
            else {
                hoodAngle += 0.1;
            }
        }
        if(gamepad1.dpadLeftWasPressed()) {
            if (gamepad1.left_bumper) {
                hoodAngle -= 0.01;
            }
            else {
                hoodAngle -= 0.1;
            }
        }
        // Clamp both values between MIN and MAX.
        flywheelSpeed = Math.max(MIN_FLYWHEEL_SPEED, Math.min(flywheelSpeed, MAX_FLYWHEEL_SPEED));
        hoodAngle = Math.max(MIN_ANGLE, Math.min(hoodAngle, MAX_ANGLE));
        if(gamepad1.left_trigger > 0) {
            turret(flywheelSpeed, hoodAngle);
        }
        else {
            turret(0, hoodAngle);
        }
    }

    // This routine drives the robot field relative
    private void driveFieldRelative(double forward, double right, double rotate) {
        // First, convert direction being asked to drive to polar coordinates
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        // Second, rotate angle by the angle the robot is pointing
        theta = AngleUnit.normalizeRadians(theta -
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        // Third, convert back to cartesian
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        // Finally, call the drive method with robot relative forward and right amounts
        drive(newForward, newRight, rotate);
    }

    // Thanks to FTC16072 for sharing this code!!
    public void drive(double forward, double right, double rotate) {
        // This calculates the power needed for each wheel based on the amount of forward,
        // strafe right, and rotate
        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower = forward + right - rotate;
        double backLeftPower = forward - right + rotate;

        double maxPower = 1.0;
        double maxSpeed = 1.0;  // make this slower for outreaches

        // This is needed to make sure we don't pass > 1.0 to any wheel
        // It allows us to keep all of the motors in proportion to what they should
        // be and not get clipped
        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));

        // We multiply by maxSpeed so that it can be set lower for outreaches
        // When a young child is driving the robot, we may not want to allow full
        // speed.
        frontLeftDrive.setPower(maxSpeed * (frontLeftPower / maxPower));
        frontRightDrive.setPower(maxSpeed * (frontRightPower / maxPower));
        backLeftDrive.setPower(maxSpeed * (backLeftPower / maxPower));
        backRightDrive.setPower(maxSpeed * (backRightPower / maxPower));
    }

    private void turret(double speed, double angle) {
        flywheel.setPower(speed);
        hood.setPosition(angle);
        telemetry.addData("Flywheel Target", "Speed %5.2f", flywheel.getTargetPosition());
        telemetry.addData("Flywheel Current", "Speed %5.2f", flywheel.getCurrentPosition());
        telemetry.addData("Hood Angle", "Angle %5.2f", hood.getPosition());
    }
}
