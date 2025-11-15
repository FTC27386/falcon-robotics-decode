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
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.geometry.Rotation2d;
import com.seattlesolvers.solverslib.geometry.Transform2d;
import com.seattlesolvers.solverslib.geometry.Vector2d;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Misc.RobotConstants;

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
@TeleOp(name = "InterpLUT Aim Solvers", group = "Robot")

public class InterpLUTAimSolvers extends OpMode {

    public static InterpLUT lut;
    public static double offsetRadians;
    public static double FLYWHEEL_SPEED;
    public static double targetX,
            targetY,
            yawMultiplier = 1;
    // This declares the four motors needed
    DcMotor flywheel1, flywheel2;
    GoBildaPinpointDriver localizer;
    Servo leftTurretServo, rightTurretServo;
    //DcMotor intake;
    DcMotor frontLeftDrive,
            frontRightDrive,
            backLeftDrive,
            backRightDrive;
    Servo hood;
    // This declares the IMU needed to get the current direction the robot is facing
    IMU imu;
    Pose2D pinpointPose_TEMP;
    Pose2d currentPose;
    Vector2d currentVector;
    Vector2d targetVector = new Vector2d();
    Transform2d poseDelta;
    Vector2d vectorDelta;
    double field_relative_angle,
            HOOD_ANGLE,
            MAX_ANGLE,
            MIN_ANGLE,
            turret_angle,
            robot_relative_angle,
            odo_turretservo_angle;

    @Override
    public void init() {
        localizer = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        localizer.setOffsets(-116, -91.751, DistanceUnit.MM);
        localizer.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        localizer.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        localizer.resetPosAndIMU();
        turret_angle = 0;
        FLYWHEEL_SPEED = 0.5;
        HOOD_ANGLE = 0.5;
        MAX_ANGLE = 1;
        MIN_ANGLE = 0;
        targetX = 0;
        targetY = 0;

        leftTurretServo = hardwareMap.get(Servo.class, "turretServo1");
        rightTurretServo = hardwareMap.get(Servo.class, "turretServo2");
        //intake = hardwareMap.get(DcMotor.class, "intake");
        //intake.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel1 = hardwareMap.get(DcMotor.class, "flywheel1");
        flywheel2 = hardwareMap.get(DcMotor.class, "flywheel2");
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
        flywheel1.setZeroPowerBehavior(FLOAT); //Makes the flywheel1 not turn itself off
        flywheel2.setZeroPowerBehavior(FLOAT);


        // imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

    @Override
    public void loop() {
        targetVector = new Vector2d(targetX, targetY);
        localizer.update();
        pinpointPose_TEMP = localizer.getPosition();
        currentPose = new Pose2d(
                pinpointPose_TEMP.getX(DistanceUnit.INCH),
                pinpointPose_TEMP.getY(DistanceUnit.INCH),
                new Rotation2d(
                        pinpointPose_TEMP.getHeading(AngleUnit.RADIANS)));

        currentVector = new Vector2d(
                currentPose.getX(),
                currentPose.getY()
        );
        
        vectorDelta = targetVector.minus(currentVector);
        //robot_relative_angle = (Math.PI/2 - vectorDelta.angle()) //get complementary angle
        //        + currentPose.getRotation().getRadians();
        robot_relative_angle = (Math.PI/2 - vectorDelta.angle()); //get complementary angle

        odo_turretservo_angle = 0.5
                - ((Math.toDegrees(robot_relative_angle)+313)/626);
        //radians of robot-relative angle * conv. to degrees * conv. to servo ticks * GR2 * GR1

        telemetry.addLine("Left trigger is shooter");
        telemetry.addLine("Dpad up/down controls hood angle");
        telemetry.addData("Absolute X", currentPose.getX());
        telemetry.addData("Absolute Y", currentPose.getY());
        telemetry.addData("Distance X", vectorDelta.getX());
        telemetry.addData("Distance Y", vectorDelta.getY());
        telemetry.addData("Distance Vector", vectorDelta.magnitude());
        telemetry.addData("Odo Turret Angle", odo_turretservo_angle);
        telemetry.addData("Robot Relative Angle", Math.toDegrees(robot_relative_angle));

        telemetry.addData("InterpLUT", lut);
        telemetry.addData("Translational Deviation", vectorDelta.angle());
        drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        if (gamepad1.optionsWasPressed()) localizer.resetPosAndIMU();
        if (gamepad1.aWasPressed()) imu.resetYaw();
        if (gamepad1.crossWasPressed()) turret_angle += .05;
        if (gamepad1.triangleWasPressed()) turret_angle -= .05;
        if (gamepad1.dpadUpWasPressed()) HOOD_ANGLE += 0.1;
        if (gamepad1.dpadDownWasPressed()) HOOD_ANGLE -= 0.1;

        // Clamp both values between MIN and MAX.
        HOOD_ANGLE = clamp(HOOD_ANGLE, MIN_ANGLE, MAX_ANGLE);
        turret_angle = clamp(turret_angle, 0, 1);
        if (gamepad1.left_trigger > 0)
            turret(FLYWHEEL_SPEED, HOOD_ANGLE, Math.abs(odo_turretservo_angle));
        else turret(0, HOOD_ANGLE, Math.abs(odo_turretservo_angle));
        if (gamepad1.shareWasPressed()) lut.add(vectorDelta.magnitude(), HOOD_ANGLE);
        //intake.setPower(gamepad1.right_trigger);
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
        flywheel1.setPower(speed);
        flywheel2.setPower(speed);
        hood.setPosition(angle);
        leftTurretServo.setPosition(turretAngle);
        rightTurretServo.setPosition(turretAngle);
        telemetry.addData("Hood Angle", "Angle %5.2f", hood.getPosition());
        telemetry.addData("Turret Angle", leftTurretServo.getPosition());
    }
}
