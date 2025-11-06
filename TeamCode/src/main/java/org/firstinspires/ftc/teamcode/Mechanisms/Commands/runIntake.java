package org.firstinspires.ftc.teamcode.Mechanisms.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Mechanisms.IntakeNoSort;

public class runIntake extends CommandBase {

    private final IntakeNoSort robot_intake;

    public runIntake(IntakeNoSort intake)
    {
        robot_intake = intake;
        addRequirements(robot_intake);
    }
    @Override
    public void initialize()
    {
        robot_intake.intake();
    }
    @Override
    public boolean isFinished()
    {
        return true;
    }
}
