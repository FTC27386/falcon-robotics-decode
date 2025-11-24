package org.firstinspires.ftc.teamcode.Mechanisms.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Mechanisms.IntakeNoSort;
import org.firstinspires.ftc.teamcode.Mechanisms.Robot;

public class stopIntake extends CommandBase {

    private final Robot robot;

    public stopIntake(Robot robot)
    {
        this.robot = robot;
    }
    @Override
    public void initialize()
    {
        robot.getI().stopIntake();
    }
    @Override
    public boolean isFinished()
    {
        return true;
    }
}
