package org.firstinspires.ftc.teamcode.Mechanisms.Commands;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Mechanisms.Robot;
import org.firstinspires.ftc.teamcode.Utility.RobotConstants;

public class oneShot extends SequentialCommandGroup {
    private Robot r;
    private RobotConstants.robotState shootingZone;


    public oneShot(Robot r)
    {
        this.r = r;
        addRequirements(r.getI(),r.getS());
        addCommands(
                new InstantCommand(() -> r.getI().close()),
                new readyShooter(r),
                new runIntake(r),
                new pulseGate(r, 150),
                new stopIntake(r)
        );
    }
}
