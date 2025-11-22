package org.firstinspires.ftc.teamcode.Mechanisms.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Mechanisms.turretShooter;

public class updateShooter extends CommandBase
{
    private final turretShooter robot_shooter;
    public updateShooter(turretShooter shooter)
    {
        robot_shooter = shooter;
    }

    @Override
    public void initialize() {
        robot_shooter.periodic();
    }
    @Override
    public boolean isFinished()
    {
        return true;
    }
}
