package org.firstinspires.ftc.teamcode.Mechanisms.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Mechanisms.IntakeNoSort;

public class updateIntake extends CommandBase {

    private final IntakeNoSort robot_intake;

    public updateIntake(IntakeNoSort intake)
    {
        robot_intake = intake;
        addRequirements(robot_intake);
    }

    @Override
    public void initialize()
    {
        robot_intake.periodic();
    }
    @Override
    public boolean isFinished()
    {
        return true;
    }
}
