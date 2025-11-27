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


import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Mechanisms.UtilMethods;

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


@TeleOp(name = "InterpLUT Aim Analog", group = "Robot")


public class InterpLUTAimAnalog extends OpMode {
    public static double kP = 0;
    public static double kD = 0;
    public static double kF = 0;
    public static double kL = 0;

    private AnalogInput turretEnc;
    public static InterpLUT lut;
    public static double offsetRadians;
    public static double FLYWHEEL_SPEED;
    public static double targetX = -144,
            targetY = 144,
            yawMultiplier = 1;
    // This declares the four motors needed
    DcMotor flywheel1, flywheel2;
    GoBildaPinpointDriver localizer;
    CRServo leftTurretServo, rightTurretServo;
    //DcMotor intake;
    DcMotor frontLeftDrive,
            frontRightDrive,
            backLeftDrive,
            backRightDrive;
    Servo hood;
    // This declares the IMU needed to get the current direction the robot is facing
    IMU imu;
    double field_adjustment_angle,
            trueX,
    trueY,
            HOOD_ANGLE,
            MAX_ANGLE,
            MIN_ANGLE,
            turret_angle,
            pinpointX,
            distanceX,
    turretDeg,
            distanceVector,
            pinpointY,
            distanceY,
    error,
            robot_relative_angle,
            previousread,
    signal,
    axonRead,
    degreeRead,
    deltaRead,
            odo_turretservo_angle;
    double degreestravelled = 0;
    int rotations=0;
    Pose2D pose;

    PIDController turretPDFL;

    @Override
    public void init() {
        turretPDFL = new PIDController(kP, 0, kD);
        turretEnc = hardwareMap.get(AnalogInput.class, "turret_encoder");
        localizer = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        localizer.setOffsets(-116, -91.751, DistanceUnit.MM);
        localizer.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        localizer.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        localizer.resetPosAndIMU();
        turret_angle = 0;
        FLYWHEEL_SPEED = 1;
        HOOD_ANGLE = 0.5;
        MAX_ANGLE = 1;
        MIN_ANGLE = 0;

        leftTurretServo = hardwareMap.get(CRServo.class, "left_turret_servo");
        rightTurretServo = hardwareMap.get(CRServo.class, "right_turret_servo");
        //intake = hardwareMap.get(DcMotor.class, "intake");
        //intake.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel1 = hardwareMap.get(DcMotor.class, "flywheel_top");
        flywheel2 = hardwareMap.get(DcMotor.class, "flywheel_bottom");
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
        flywheel1.setDirection(DcMotor.Direction.FORWARD);
        flywheel2.setDirection(DcMotor.Direction.REVERSE);
        flywheel1.setZeroPowerBehavior(FLOAT); //Makes the flywheel1 not turn itself off
        flywheel2.setZeroPowerBehavior(FLOAT);

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

        //encoder test
        turretPDFL.setPID(kP, 0, kD);
        axonRead = turretEnc.getVoltage();
        degreeRead = axonRead * (360/3.3);
        deltaRead = degreeRead - previousread;
        if (deltaRead > 180)
        {
            rotations -= 1;
        }
        if (deltaRead < -180)
        {
            rotations += 1;
        }
        degreestravelled = rotations*360.0 + degreeRead;
        turretDeg = degreestravelled*(5.0)*(60/170.0);
        error = UtilMethods.AngleDifference(odo_turretservo_angle, turretDeg) ;
        signal = turretPDFL.calculate(-error, 0);
        signal += Math.signum(signal) * kL;






        localizer.update();
        pose = localizer.getPosition();
        pinpointX = pose.getX(DistanceUnit.INCH); //VARIABLES USE STANDARD CARTESIAN AXES!!!
        trueY = pinpointX;
        pinpointY = pose.getY(DistanceUnit.INCH);
        trueX = -pinpointY;
        distanceX = targetX - trueX;
        distanceY = targetY - trueY;
        field_adjustment_angle = (90 - Math.toDegrees(Math.atan2(distanceY,distanceX)));
       // odo_turretservo_angle = 1-(localizer.getHeading(AngleUnit.RADIANS)+Math.toRadians(313))/Math.toRadians(626);
        //odo_turretservo_angle +=(Math.toRadians(field_adjustment_angle))/Math.toRadians(626);
       // odo_turretservo_angle *= 360;
        odo_turretservo_angle = -pose.getHeading(AngleUnit.DEGREES) - field_adjustment_angle;



        //distanceVector = Math.sqrt(Math.pow(distanceX, 2) + Math.pow(distanceY, 2));
        //distanceVector = Math.atan2(distanceX, distanceY);
        //odo_turretservo_angle = gamepad1.right_trigger;
        //field_relative_angle = Math.atan2(distanceX, distanceY); //Inverted here because X is actually the vertical axis
        //robot_relative_angle = ((Math.PI/2 - field_relative_angle) - ((yawMultiplier * localizer.getHeading(AngleUnit.RADIANS))));
        //odo_turretservo_angle = 0.5 -(-Math.toDegrees(robot_relative_angle) * ((double) 1 / 355) * ((double) 170 / 60) * ((double) 1 / 5));
        //radians of robot-relative angle * conv. to degrees * conv. to servo ticks * GR2 * GR1

        telemetry.addLine("Left trigger is shooter");
        telemetry.addLine("Dpad up/down controls hood angle");
        telemetry.addData("Attempted Position", turret_angle);
        telemetry.addData("Heading", localizer.getHeading(AngleUnit.DEGREES));
        telemetry.addData("goal field angle", field_adjustment_angle);
        telemetry.addData("odo ang",odo_turretservo_angle);
        telemetry.addData("x to goal", distanceX);
        telemetry.addData("y to goal", distanceY);
        telemetry.addData("PINPOINT Y (horiz axis)", pinpointY);
        telemetry.addData("PINPOINT X (vert)", pinpointX);
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
        if (gamepad1.left_trigger > 0) turret(FLYWHEEL_SPEED, HOOD_ANGLE, gamepad1.left_trigger);
        else if (gamepad1.right_trigger >0)
        {
            turret(0,0,-gamepad1.right_trigger);
        }
         else {
            turret(0, HOOD_ANGLE, signal);
        }
        if (gamepad1.shareWasPressed()) lut.add(distanceVector, HOOD_ANGLE);
        //intake.setPower(gamepad1.right_trigger);
        previousread = degreeRead;
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

    private void turret(double speed, double angle, double turretPower) {
        flywheel1.setPower(speed);
        flywheel2.setPower(speed);
        leftTurretServo.setPower(turretPower);
        rightTurretServo.setPower(turretPower);
        hood.setPosition(angle);
        telemetry.addData("degrees Travelled", degreestravelled);
        telemetry.addData("current servo pos", degreeRead);
        telemetry.addData("pose_rotation", pose.getHeading(AngleUnit.DEGREES));
        telemetry.addData("error", error);
        telemetry.addData("odo_correction_ticks",odo_turretservo_angle);
        telemetry.addData("field_correction_ticks", (-Math.toRadians(field_adjustment_angle))/Math.toRadians(626));
        telemetry.addData("rotations", rotations);
        telemetry.addData("turretDeg", turretDeg);
    }
}
