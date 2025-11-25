package org.firstinspires.ftc.teamcode.Mechanisms.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.Mechanisms.Robot;

public class defaultDrive extends CommandBase {

    Robot r;
    GamepadEx gp;

    public defaultDrive(Robot r, GamepadEx gp)
        {
            this.r = r;
        }

        @Override
    public void execute()
        {
            r.getD().teleOpDrive(gp);
        }

}
