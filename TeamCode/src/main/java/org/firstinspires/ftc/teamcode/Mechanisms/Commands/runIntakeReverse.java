package org.firstinspires.ftc.teamcode.Mechanisms.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Mechanisms.Robot;

public class runIntakeReverse extends CommandBase {

    private final Robot r;

    public runIntakeReverse(Robot r)
    {
        this.r = r;
        addRequirements(r.getI());
    }
    @Override
    public void initialize()
    {
        r.getI().outtake();
    }
    @Override
    public boolean isFinished()
    {
        return true;
    }
}
