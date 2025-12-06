package org.firstinspires.ftc.teamcode.Mechanisms.Commands;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Mechanisms.Robot;
import org.firstinspires.ftc.teamcode.Utility.RobotConstants;

public class closeCycle extends SequentialCommandGroup {

    Robot r;

    public closeCycle(Robot r)
    {
        this.r=r;
        addRequirements(r.getI(),r.getS());
        addCommands(
                new InstantCommand(()-> r.getI().close()),
                new readyShooter(r),
                new InstantCommand(()->r.getI().open()),
                new runIntakeTimed(r,900),
                new stopIntake(r)
        );
    }
}
