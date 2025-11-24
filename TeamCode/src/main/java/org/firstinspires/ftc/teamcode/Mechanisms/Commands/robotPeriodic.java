package org.firstinspires.ftc.teamcode.Mechanisms.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Mechanisms.Robot;

public class robotPeriodic extends CommandBase {
    private final Robot robot;

    public robotPeriodic(Robot r)
    {
        this.robot = r;
    }

    @Override
    public void initialize()
    {
        robot.periodic();
    }
}
