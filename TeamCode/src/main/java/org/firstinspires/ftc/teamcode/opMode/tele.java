package org.firstinspires.ftc.teamcode.opMode;

import com.bylazar.gamepad.Gamepad;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.button.Button;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Mechanisms.Commands.readyShooter;
import org.firstinspires.ftc.teamcode.Mechanisms.Commands.robotPeriodic;
import org.firstinspires.ftc.teamcode.Mechanisms.Robot;
import org.firstinspires.ftc.teamcode.Misc.RobotConstants;

public class tele extends CommandOpMode {
    Button shootFar;
    Button shootClose;
    GamepadEx driverOp;
    private Robot r;
    @Override
    public void initialize()
    {

        r = new Robot(hardwareMap);
        register(r);
        r.setDefaultCommand(new robotPeriodic(r));

        driverOp = new GamepadEx(gamepad1);
        shootFar = driverOp.getGamepadButton(GamepadKeys.Button.X);
        shootClose = driverOp.getGamepadButton(GamepadKeys.Button.CIRCLE);
        shootFar.whenPressed(new readyShooter(r, RobotConstants.robotState.FAR_SHOOT));
        shootClose.whenPressed(new readyShooter(r, RobotConstants.robotState.CLOSE_SHOOT));



    }
}
