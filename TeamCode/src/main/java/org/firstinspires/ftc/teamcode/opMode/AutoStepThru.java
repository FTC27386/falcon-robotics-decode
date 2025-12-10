package org.firstinspires.ftc.teamcode.opMode;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.button.Button;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Mechanisms.Commands.climb;
import org.firstinspires.ftc.teamcode.Mechanisms.Commands.defaultDrive;
import org.firstinspires.ftc.teamcode.Mechanisms.Commands.followPath;
import org.firstinspires.ftc.teamcode.Mechanisms.Commands.magDump;
import org.firstinspires.ftc.teamcode.Mechanisms.Commands.runIntake;
import org.firstinspires.ftc.teamcode.Mechanisms.Commands.runIntakeTimed;
import org.firstinspires.ftc.teamcode.Mechanisms.Commands.stopIntake;
import org.firstinspires.ftc.teamcode.Mechanisms.Paths;
import org.firstinspires.ftc.teamcode.Mechanisms.Robot;

import java.util.function.Supplier;

@Config
@TeleOp(name = "Manual Stepping Thru Auto")
public class AutoStepThru extends CommandOpMode {
    int index = 0;
    Button intake;
    Button relocalize;
    Button shoot;
    Button changeTarget;
    GamepadEx driverOp;
    Button climb;
    Button dpadup;
    Button dpaddown;
    public static double hood_pos = 0;
    public static double flywheel_speed = -1500;
    private Robot r;
    Paths paths;
    public Command[] AutoCommands =
            {
                    new InstantCommand(()-> r.setShooterValues()),
                    new InstantCommand(()-> r.getI().close()),

                    new followPath(r, paths.Path0),
                    new magDump(r),
                    new runIntake(r),
                    new InstantCommand(()-> r.setShooterValues()),
                    new followPath(r, paths.Path1), //intake 1st line
                    new stopIntake(r),
                    new followPath(r, paths.Path2), //return to shoot point
                    new magDump(r),
                    new followPath(r, paths.Path3),
                    new runIntake(r),
                    new InstantCommand(()-> r.setShooterValues()),
                    new followPath(r, paths.Path4),
                    new stopIntake(r),
                    new followPath(r, paths.Path5),
                    new magDump(r),
                    new followPath(r, paths.Path6),
                    new runIntake(r),
                    new InstantCommand(()-> r.setShooterValues()),
                    new followPath(r, paths.Path7),
                    new stopIntake(r),
                    new followPath(r, paths.Path8),
                    new magDump(r),
                    new followPath(r, paths.Path9)
            };

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

        intake = driverOp.getGamepadButton(GamepadKeys.Button.SQUARE);
        relocalize = driverOp.getGamepadButton(GamepadKeys.Button.OPTIONS);
        changeTarget = driverOp.getGamepadButton(GamepadKeys.Button.SHARE);
        shoot = driverOp.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER);
        climb = driverOp.getGamepadButton(GamepadKeys.Button.TOUCHPAD);
        paths = new Paths(r.getD().follower);

        climb.whenPressed(new climb(r));
        intake.whenPressed(new runIntakeTimed(r, 2000));
        relocalize.whenPressed(new InstantCommand(() -> r.getD().reloc(new Pose(8, 8, Math.toRadians(90)))));
        changeTarget.whenPressed(new InstantCommand(() -> r.getD().relocTarget(
               new Pose(
                       r.getD().getCurrentPose().getX() - 12
                       ,r.getD().getCurrentPose().getY() + 12
                       ,Math.toRadians(90)
               ))));
        schedule(new InstantCommand(() -> r.getD().follower.startTeleOpDrive()));
        schedule(new RunCommand(() -> r.getS().setTurretPosition(r.getD().yoCalcAim())));
        shoot.whenPressed(new magDump(r));
    }

    @Override
    public void run() {

        if (gamepad1.dpadUpWasPressed())
        {
                    schedule(AutoCommands[index]);
        }
        if(gamepad1.dpadLeftWasPressed())
        {
            index -= 1;
        }
        if(gamepad1.dpadRightWasPressed())
        {
            index +=1;
        }


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
