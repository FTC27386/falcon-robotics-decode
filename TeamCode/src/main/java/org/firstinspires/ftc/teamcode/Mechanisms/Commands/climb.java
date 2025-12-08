package org.firstinspires.ftc.teamcode.Mechanisms.Commands;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Mechanisms.Paths;
import org.firstinspires.ftc.teamcode.Mechanisms.Robot;

public class climb extends SequentialCommandGroup {

    Robot r;

    Paths paths;
    public climb(Robot r)
    {
        paths = new Paths(r.getD().follower);
        this.r = r;
        addCommands(
                new followPath(r,paths.park),
                new InstantCommand(()->r.getL().setActivated(true)),
                new InstantCommand(()->r.getL().unlatch()),
                new WaitCommand(300),
                new InstantCommand(()->r.getL().down())
        );
    }

}
