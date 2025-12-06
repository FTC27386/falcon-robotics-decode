package org.firstinspires.ftc.teamcode.opMode;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.button.Button;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Mechanisms.Commands.defaultDrive;
import org.firstinspires.ftc.teamcode.Mechanisms.Commands.runIntakeTimed;
import org.firstinspires.ftc.teamcode.Mechanisms.Commands.toggleIntake;
import org.firstinspires.ftc.teamcode.Mechanisms.Commands.readyShooter;
import org.firstinspires.ftc.teamcode.Mechanisms.Robot;
import org.firstinspires.ftc.teamcode.Utility.RobotConstants;

import java.util.function.Supplier;

@TeleOp(name = "TeleOp")
public class teleOp extends CommandOpMode {

    Button shootFar;
    Button shootClose;
    Button intake;
    Button relocalize;
    GamepadEx driverOp;
    private Robot r;
    @Override
    public void initialize()
    {
        super.reset();
        r = new Robot(hardwareMap);
        register(r.getS(), r.getD(), r.getI());
        driverOp = new GamepadEx(gamepad1);
        Supplier<Double> leftX = driverOp::getLeftX;
        Supplier<Double> leftY = driverOp::getLeftY;
        Supplier<Double> rightX = driverOp::getRightX;
        r.getD().setDefaultCommand(new defaultDrive(r,leftY, leftX, rightX));

        shootFar = driverOp.getGamepadButton(GamepadKeys.Button.X);
        shootClose = driverOp.getGamepadButton(GamepadKeys.Button.CIRCLE);
        intake = driverOp.getGamepadButton(GamepadKeys.Button.SQUARE);
        relocalize = driverOp.getGamepadButton(GamepadKeys.Button.OPTIONS);

        shootFar.whenPressed(new readyShooter(r, RobotConstants.robotState.FAR_SHOOT));
        shootClose.whenPressed(new readyShooter(r, RobotConstants.robotState.CLOSE_SHOOT));
        intake.whenPressed(new runIntakeTimed(r,2000));
        relocalize.whenPressed(new InstantCommand(()->r.getD().reloc(new Pose(8,8,Math.toRadians(90)))));
        schedule(new InstantCommand(()-> r.getD().follower.startTeleOpDrive()));

    }
    @Override
    public void run()
    {
        telemetry.addData("x:", r.getD().x);
        telemetry.addData("y:", r.getD().y);
        telemetry.addData("heading",r.getD().heading);
        telemetry.addData("adjustment",r.getD().yoCalcAim());
        telemetry.update();
        super.run();
    }

}
