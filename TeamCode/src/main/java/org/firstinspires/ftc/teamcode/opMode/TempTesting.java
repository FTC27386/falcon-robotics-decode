package org.firstinspires.ftc.teamcode.opMode;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.button.Button;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Mechanisms.Commands.autoCloseShotBlue;
import org.firstinspires.ftc.teamcode.Mechanisms.Commands.defaultDrive;
import org.firstinspires.ftc.teamcode.Mechanisms.Paths;
import org.firstinspires.ftc.teamcode.Mechanisms.PathsMirrored;
import org.firstinspires.ftc.teamcode.Mechanisms.Robot;

import java.util.function.Supplier;

@Config
@TeleOp(name = "TeleOp Test Ver.")
public class TempTesting extends CommandOpMode {

    Button Triangle;
    Button Square;
    Button Circle;
    Button Cross;
    Button dpad_up;
    GamepadEx dpad_down;
    Button dpad_left;
    Button dpad_right;
    public static double hood_pos = 0;
    public static double flywheel_speed = -1500;
    private Robot r;
    Paths paths;
    PathsMirrored paths_mirrored;
    GamepadEx driverOp;

    @Override
    public void initialize() {

        super.reset();
        r = new Robot(hardwareMap);
        register(r.getS(), r.getD(), r.getI(), r.getL());
        driverOp = new GamepadEx(gamepad1);
        Supplier<Double> leftX = driverOp::getLeftX;
        Supplier<Double> leftY = driverOp::getLeftY;
        Supplier<Double> rightX = driverOp::getRightX;
        r.getD().setDefaultCommand(new defaultDrive(r, leftY, leftX, rightX));
        Paths paths = new Paths(r.getD().follower);
        PathsMirrored mirroredPaths = new PathsMirrored(r.getD().follower);


        Square = driverOp.getGamepadButton(GamepadKeys.Button.SQUARE);
        Circle = driverOp.getGamepadButton(GamepadKeys.Button.CIRCLE);
        Triangle = driverOp.getGamepadButton(GamepadKeys.Button.TRIANGLE);
        Cross = driverOp.getGamepadButton(GamepadKeys.Button.CROSS);
        dpad_up = driverOp.getGamepadButton(GamepadKeys.Button.DPAD_UP);
        dpad_left = driverOp.getGamepadButton(GamepadKeys.Button.DPAD_LEFT);
        dpad_right = driverOp.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT);

        dpad_right.whenPressed(new autoCloseShotBlue(r));
        Square.whenPressed(new InstantCommand(()->r.getD().reloc(new Pose(
                8,8,Math.toRadians(90)
        ))));

        Circle.whenPressed(new InstantCommand(()->r.getL().latch()));
        Cross.whenPressed(new InstantCommand(()->r.getL().unlatch()));

        schedule(new InstantCommand(() -> r.getD().follower.startTeleOpDrive()));
        schedule(new RunCommand(() -> r.getS().setTurretPosition(r.getD().yoCalcAim())));

    }

    @Override
    public void run() {
        telemetry.addData("x:", r.getD().x);
        telemetry.addData("y:", r.getD().y);
        telemetry.addData("heading", r.getD().getCurrentPose().getHeading());
        telemetry.addData("adjustment", r.getD().yoCalcAim());
        telemetry.addData("flywheel target velocity", r.getS().getSpeedControl().getSetPoint());
        telemetry.addData("flywheel error", r.getS().getSpeedControl().getPositionError());
        telemetry.addData("flywheel speed", r.getS().getCurrentSpeed());
        telemetry.addData("Hood", r.getD().yoCalcHood());
        telemetry.addData("flywheel response", r.getS().getFlywheelSignal());
        telemetry.addData("turret ticks", r.getS().getTurretPosition());
        telemetry.addData("lift power", r.getL().getPIDResponse());
        telemetry.addData("lift pose", r.getL().getLiftPose());
        telemetry.addData("Distance", r.getD().yoCalcDist());

        telemetry.update();
        super.run();
    }

}
