package org.firstinspires.ftc.teamcode.opMode;

import com.bylazar.gamepad.Gamepad;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.button.Button;

import org.firstinspires.ftc.teamcode.Mechanisms.Commands.robotPeriodic;
import org.firstinspires.ftc.teamcode.Mechanisms.Robot;

public class tele extends CommandOpMode {
    private Robot r;
    Gamepad driver1;
    Button cross;
    @Override
    public void initialize()
    {
        r = new Robot(hardwareMap);
        register(r);
        r.setDefaultCommand(new robotPeriodic(r));
    }
}
