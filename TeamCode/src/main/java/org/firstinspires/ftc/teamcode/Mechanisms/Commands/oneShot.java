package org.firstinspires.ftc.teamcode.Mechanisms.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.CommandGroupBase;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Mechanisms.Robot;
import org.firstinspires.ftc.teamcode.Misc.RobotConstants;

public class oneShot extends SequentialCommandGroup {
    private  Robot r;
    private RobotConstants.robotState shootingZone;


    public oneShot(Robot r, RobotConstants.robotState dist)
    {
        this.shootingZone = dist;
        this.r = r;
        addRequirements(r.getI(),r.getS());
        addCommands(
                new InstantCommand(() -> r.getI().close()),
                new readyShooter(r, dist),
                new runIntake(r),
                new pulseGate(r, 150),
                new WaitCommand(200),
                new stopIntake(r)
        );
    }
}
