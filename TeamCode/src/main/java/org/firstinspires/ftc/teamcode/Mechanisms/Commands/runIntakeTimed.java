package org.firstinspires.ftc.teamcode.Mechanisms.Commands;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Mechanisms.IntakeNoSort;

public class runIntakeTimed extends SequentialCommandGroup {

    public ElapsedTime timer;

    public runIntakeTimed(IntakeNoSort intake, int time)
    {
        addCommands(
                new runIntake(intake),
                new WaitCommand(time),
                new stopIntake(intake)
        );
        addRequirements(intake);
    }
}
