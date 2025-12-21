package org.firstinspires.ftc.teamcode.opMode;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.button.Button;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Mechanisms.Commands.followPath;
import org.firstinspires.ftc.teamcode.Mechanisms.Commands.defaultDrive;
import org.firstinspires.ftc.teamcode.Mechanisms.Commands.liftoff;
import org.firstinspires.ftc.teamcode.Mechanisms.Commands.magDump;
import org.firstinspires.ftc.teamcode.Mechanisms.Commands.runIntakeReverseTimed;
import org.firstinspires.ftc.teamcode.Mechanisms.Commands.runIntakeTimed;
import org.firstinspires.ftc.teamcode.Mechanisms.Paths;
import org.firstinspires.ftc.teamcode.Mechanisms.PathsMirrored;
import org.firstinspires.ftc.teamcode.Mechanisms.Robot;
import org.firstinspires.ftc.teamcode.Utility.RobotConstants;

import java.util.function.Supplier;

@Config
@TeleOp(name = "TeleOp")
public class teleOp extends CommandOpMode {
    Button revShooter;
    Button intake;
    Button relocalizeBlue;
    Button relocalizeRed;
    Button shoot;
    Button outtake;
    Button changeTarget;
    GamepadEx driverOp;
    GamepadEx secondDriverOp;
    Button goToClimb;
    Button park;
    Button stopAllMech;
    Button addOffsetTurret;
    Button subtractOffsetTurret;
    Button defaultOffsetTurret;
    Button addOffsetLift;
    Button subtractOffsetLift;
    Button defaultOffsetLift;
    Button stowIntake;
    Button deployIntake;

    public static double hood_pos = 0;
    public static double flywheel_speed = -1500;
    private Robot r;
    Paths paths;
    PathsMirrored paths_mirrored;


    @Override
    public void initialize() {
        super.reset();  //boilerplate including pedropathing setup
        r = new Robot(hardwareMap);
        register(r.getS(), r.getD(), r.getI(), r.getL());
        driverOp = new GamepadEx(gamepad1);
        secondDriverOp = new GamepadEx(gamepad2);
        Supplier<Double> leftX = driverOp::getLeftX;
        Supplier<Double> leftY = driverOp::getLeftY;
        Supplier<Double> rightX = driverOp::getRightX;
        r.getD().setDefaultCommand(new defaultDrive(r, leftY, leftX, rightX));
        Paths paths = new Paths(r.getD().follower);
        PathsMirrored mirroredPaths = new PathsMirrored(r.getD().follower);

        //button binds
        intake = driverOp.getGamepadButton(GamepadKeys.Button.SQUARE);
        outtake = driverOp.getGamepadButton(GamepadKeys.Button.CIRCLE);
        relocalizeBlue = driverOp.getGamepadButton(GamepadKeys.Button.OPTIONS);
        relocalizeRed = secondDriverOp.getGamepadButton(GamepadKeys.Button.OPTIONS);
        changeTarget = driverOp.getGamepadButton(GamepadKeys.Button.SHARE);
        goToClimb = driverOp.getGamepadButton(GamepadKeys.Button.TOUCHPAD);
        park = driverOp.getGamepadButton(GamepadKeys.Button.DPAD_UP);
        revShooter = driverOp.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER);
        stopAllMech = driverOp.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON);

        shoot = secondDriverOp.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER);
        addOffsetTurret = secondDriverOp.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT);
        subtractOffsetTurret = secondDriverOp.getGamepadButton(GamepadKeys.Button.DPAD_LEFT);
        defaultOffsetTurret = secondDriverOp.getGamepadButton(GamepadKeys.Button.SHARE);
        addOffsetLift = secondDriverOp.getGamepadButton(GamepadKeys.Button.TRIANGLE);
        subtractOffsetLift = secondDriverOp.getGamepadButton(GamepadKeys.Button.CROSS);
        defaultOffsetLift = secondDriverOp.getGamepadButton(GamepadKeys.Button.DPAD_LEFT);
        stowIntake = secondDriverOp.getGamepadButton(GamepadKeys.Button.SQUARE);
        deployIntake = secondDriverOp.getGamepadButton(GamepadKeys.Button.CIRCLE);




        goToClimb.whenPressed(new SequentialCommandGroup( //test whether climb restarts teleop drive
                new followPath(r,
                RobotConstants.current_color == null || RobotConstants.current_color == RobotConstants.ALLIANCE_COLOR.RED?
                        paths.park : mirroredPaths.park),
                new InstantCommand(()->r.getD().follower.startTeleOpDrive())));
        intake.whenPressed(new runIntakeTimed(r, 2000));
        outtake.whenPressed(new runIntakeReverseTimed(r, 100));
        relocalizeBlue.whenPressed(new InstantCommand(() -> r.getD().reloc(new Pose(8, 8, Math.toRadians(90)))));
        relocalizeRed.whenPressed(new InstantCommand(() -> r.getD().reloc(new Pose(136, 8, Math.toRadians(90)))));
        changeTarget.whenPressed(new InstantCommand(() -> r.getD().relocTarget(
               new Pose(
                       r.getD().getCurrentPose().getX() + Math.cos(r.getD().getCurrentPose().getHeading() * 12)
                       ,r.getD().getCurrentPose().getY() + Math.sin(r.getD().getCurrentPose().getHeading() * 12)
               ))));

        shoot.whenPressed(new magDump(r));
        park.whenPressed(new liftoff(r));
        revShooter.whenPressed(new InstantCommand(()-> r.getS().setSpeed(-1600)));
        stopAllMech.whenPressed(new InstantCommand(()->r.getS().setSpeed(0))); //stops everything ahead of the climb sequence
        stopAllMech.whenPressed(new InstantCommand(()->r.getI().stopIntake()));
        //Offsetting the turret via a button
        addOffsetTurret.whenPressed(new InstantCommand(()->r.getS().nudgeOffset(5)));
        subtractOffsetTurret.whenPressed(new InstantCommand(()->r.getS().nudgeOffset(-5)));
        defaultOffsetTurret.whenPressed(new InstantCommand(()->r.getS().zeroOffset()));
        //offsetting the lift via button
        addOffsetLift.whenPressed(new InstantCommand(()->r.getL().nudgeLift(50)));
        subtractOffsetLift.whenPressed(new InstantCommand(()->r.getL().nudgeLift(-50)));
        defaultOffsetLift.whenPressed(new InstantCommand(()->r.getL().resetOffset()));
        stowIntake.whenPressed(new InstantCommand(()->r.getI().stow()));
        deployIntake.whenPressed(new InstantCommand(()->r.getI().deploy()));

        schedule(new InstantCommand(() -> r.getD().follower.startTeleOpDrive()));
        schedule(new RunCommand(() -> r.getS().setTurretPosition(r.getD().yoCalcAim())));
        schedule(new RunCommand(() -> r.getS().setHoodPosition(r.getD().yoCalcHood())));
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
        telemetry.addData("Distance", r.getD().yoCalcDist());
        telemetry.addData("target X", r.getD().getTarg().getX());
        telemetry.addData("target Y", r.getD().getTarg().getY());
        telemetry.addData("In zone", r.getD().inZone());
        telemetry.addData("alliance color?", RobotConstants.current_color);

        telemetry.update();
        super.run();
    }

}
