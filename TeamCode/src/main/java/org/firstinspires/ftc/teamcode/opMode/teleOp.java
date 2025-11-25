package org.firstinspires.ftc.teamcode.opMode;

import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.button.Button;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Mechanisms.Commands.defaultDrive;
import org.firstinspires.ftc.teamcode.Mechanisms.Commands.enterIntakeMode;
import org.firstinspires.ftc.teamcode.Mechanisms.Commands.readyShooter;
import org.firstinspires.ftc.teamcode.Mechanisms.Robot;
import org.firstinspires.ftc.teamcode.Misc.RobotConstants;

public class teleOp extends CommandOpMode {
    Button shootFar;
    Button shootClose;
    Button intake;
    GamepadEx driverOp;
    private Robot r;
    @Override
    public void initialize()
    {
        super.reset();
        r = new Robot(hardwareMap);
        register(r.getS(), r.getD(), r.getI());
        r.getD().setDefaultCommand(new defaultDrive(r,driverOp));
        driverOp = new GamepadEx(gamepad1);

        shootFar = driverOp.getGamepadButton(GamepadKeys.Button.X);
        shootClose = driverOp.getGamepadButton(GamepadKeys.Button.CIRCLE);
        intake = driverOp.getGamepadButton(GamepadKeys.Button.SQUARE);

        shootFar.whenPressed(new readyShooter(r, RobotConstants.robotState.FAR_SHOOT));
        shootClose.whenPressed(new readyShooter(r, RobotConstants.robotState.CLOSE_SHOOT));
        intake.whenPressed(new enterIntakeMode(r));
    }
    @Override
    public void run()
    {
        super.run();
    }

}
