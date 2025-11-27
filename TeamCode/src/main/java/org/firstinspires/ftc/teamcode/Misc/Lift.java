
        package org.firstinspires.ftc.teamcode.Misc;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "Lift", group = "Robot")
public class Lift extends LinearOpMode {

    ElapsedTime liftTimer = new ElapsedTime();
    int liftPosition = 0;

    ElapsedTime stepTimer = new ElapsedTime();
    int step = 20;

    ElapsedTime bruhTimer = new ElapsedTime();


    @Override
    public void runOpMode() {

        DcMotor liftMotor = hardwareMap.get(DcMotor.class,"lift_motor");
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setPower(1);
        liftMotor.setTargetPosition(0);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()){
            if (gamepad1.triangle & liftTimer.milliseconds() > 50){
                liftPosition += step;
                liftTimer.reset();
            }
            if (gamepad1.cross & liftTimer.milliseconds() > 50){
                liftPosition -= step;
                liftTimer.reset();
            }

            if (gamepad1.dpad_up & stepTimer.milliseconds() > 50){
                step += 10;
                stepTimer.reset();
            }
            if (gamepad1.dpad_down & stepTimer.milliseconds() > 50){
                step -= 10;
                stepTimer.reset();
            }

            if (gamepad1.right_bumper & bruhTimer.seconds() > 10){
                liftPosition = 13500;
                bruhTimer.reset();
            }
            if (gamepad1.left_bumper & bruhTimer.seconds() > 10){
                liftPosition = 0;
                bruhTimer.reset();
            }

            liftMotor.setTargetPosition(liftPosition);

            telemetry.addData("liftPosition",liftPosition);
            telemetry.addData("step", step);
            telemetry.update();
        }
    }
}