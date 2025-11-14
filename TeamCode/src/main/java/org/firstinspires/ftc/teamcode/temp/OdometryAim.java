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

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

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
@TeleOp(name = "Extremely Temporary OpMode", group = "Robot")

public class OdometryAim extends OpMode {
    // This declares the four motors needed
    DcMotor[] flywheel = new DcMotor[2];
    GoBildaPinpointDriver localizer;
    Servo leftTurretServo;
    Servo rightTurretServo;
    DcMotor intake;
    // DcMotor flywheel;
    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;
    Servo hood;

    // This declares the IMU needed to get the current direction the robot is facing
    IMU imu;

    public static double offsetRadians;
    double field_relative_angle;
    public static double FLYWHEEL_SPEED;
    double HOOD_ANGLE;
    double MAX_FLYWHEEL_SPEED;
    double MIN_FLYWHEEL_SPEED;
    double MAX_ANGLE;
    double MIN_ANGLE;
    double turret_angle;
    public static double targetX;
    public static double targetY;
    public static double yawMultiplier = 1;
    double pinpointX;
    double distanceX;
    double pinpointY;
    double distanceY;
    double robot_relative_angle;
    double odo_turretservo_angle;

    @Override
    public void init() {

        localizer = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        localizer.setOffsets(45.5, -26.251, DistanceUnit.MM );
        localizer.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        localizer.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        localizer.resetPosAndIMU();
        turret_angle = 0;
        FLYWHEEL_SPEED = 1;
        HOOD_ANGLE = 0.5;
        MAX_FLYWHEEL_SPEED = 1;
        MIN_FLYWHEEL_SPEED = 0;
        MAX_ANGLE = 1;
        MIN_ANGLE = 0;


        leftTurretServo = hardwareMap.get(Servo.class, "turretServo1");
        rightTurretServo = hardwareMap.get(Servo.class, "turretServo2");
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel[0] = hardwareMap.get(DcMotor.class, "flywheel1");
        flywheel[1] = hardwareMap.get(DcMotor.class, "flywheel2");
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
        //intake.setDirection(DcMotor.Direction.REVERSE);
        //flywheel.setZeroPowerBehavior(FLOAT); //Makes the flywheel1 not turn itself off

        //imu = hardwareMap.get(IMU.class, "imu");
        // This needs to be changed to match the orientation on your robot
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        RevHubOrientationOnRobot orientationOnRobot = new
                RevHubOrientationOnRobot(logoDirection, usbDirection);
       // imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

    @Override
    public void loop() {
        localizer.update();
        pinpointX = localizer.getPosX(DistanceUnit.INCH);
        pinpointY = localizer.getPosY(DistanceUnit.INCH);
        distanceX = pinpointX - targetX;
        distanceY = pinpointY - targetY;
        field_relative_angle = Math.atan2(distanceX, distanceY); //Inverted here because X is actually the vertical axis
        robot_relative_angle = (field_relative_angle + offsetRadians) - ((yawMultiplier* localizer.getHeading(AngleUnit.RADIANS)));
        odo_turretservo_angle = Math.toDegrees(robot_relative_angle) * ((double) 1 /355) * ((double) 170 /60) * ((double) 1 /5);

        //radians of robot-relative angle * conv. to degrees * conv. to servo ticks * GR2 * GR1

        telemetry.addLine("Press A to reset Yaw");
        telemetry.addLine("The left joystick sets the robot direction");
        telemetry.addLine("Moving the right joystick left and right turns the robot");
        telemetry.addLine("Right trigger runs intake forward, right bumper runs intake backward");
        telemetry.addLine("Left trigger turns flywheel1 on");
        telemetry.addLine("Dpad up increases flywheel1 speed, dpad down decreases flywheel1 speed");
        telemetry.addLine("Dpad left/right controls hood angle");
        telemetry.addLine("Using dpad while holding down left bumper allows finer control");
        telemetry.addData("turret attempted", odo_turretservo_angle);
        telemetry.addData("angle of deviation", Math.toDegrees(robot_relative_angle));
        telemetry.addData("turrServo1", leftTurretServo.getPosition());
        telemetry.addData("turrServo2", rightTurretServo.getPosition());

        if (gamepad1.options)
        {
            localizer.resetPosAndIMU();
        }
        // If you press the A button, then you reset the Yaw to be zero from the way
        // the robot is currently pointing
        if (gamepad1.a) {
            imu.resetYaw();
        }
        drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        /*
        if(gamepad1.right_trigger > 0) {
            intake.setPower(1);
        }
        else if (gamepad1.right_bumper) {
            intake.setPower(-1);
        }
        else {
            intake.setPower(0);
        }
         */
        if(gamepad1.crossWasPressed())
        {
            turret_angle += .05;
        }
        if(gamepad1.triangleWasPressed())
        {
            turret_angle -= .05;
        }

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
        // Clamp both values between MIN and MAX.
        FLYWHEEL_SPEED = clamp(FLYWHEEL_SPEED, MIN_FLYWHEEL_SPEED, MAX_FLYWHEEL_SPEED);
        HOOD_ANGLE = clamp(HOOD_ANGLE, MIN_ANGLE, MAX_ANGLE);
        turret_angle = clamp(turret_angle, 0, 1);
        if(gamepad1.left_trigger > 0) {
            turret(FLYWHEEL_SPEED, HOOD_ANGLE, Math.abs(odo_turretservo_angle));
        }
        else {
            turret(0, HOOD_ANGLE, Math.abs(odo_turretservo_angle));
        }

        intake.setPower(gamepad1.right_trigger);
    }

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

    private void turret(double speed, double angle, double turretAngle) {
        for (DcMotor motor : flywheel)
        {
            motor.setPower(speed);
        }
        hood.setPosition(angle);
        leftTurretServo.setPosition(turretAngle);
        rightTurretServo.setPosition(turretAngle);
        telemetry.addData("Hood Angle", "Angle %5.2f", hood.getPosition());
        telemetry.addData("TurretAngle", hood.getPosition());
    }
}
