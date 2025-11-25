package org.firstinspires.ftc.teamcode.Mechanisms.Commands;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.Mechanisms.Robot;

public class enterIntakeMode extends ParallelCommandGroup {

    Robot r;

    public enterIntakeMode(Robot r)
    {
        this.r = r;
        addRequirements(r.getI(), r.getS());
        addCommands(
                new InstantCommand(()->r.getS().setSpeed(0)),
                new runIntake((r))
        );
    }

}
