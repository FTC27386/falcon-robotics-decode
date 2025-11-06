package org.firstinspires.ftc.teamcode.Mechanisms.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Mechanisms.turretShooter;

public class readyShooter extends CommandBase{

    private final turretShooter robot_shooter;
    double speed;
    double turret;
    double hood;

    public readyShooter(turretShooter shooter, double speed, double turret, double hood)
    {
        robot_shooter = shooter;
        addRequirements(robot_shooter);
        this.speed = speed;
        this.turret = turret;
        this.hood = hood;
    }
    @Override
    public void initialize()
    {
        robot_shooter.primeShooter(speed, turret, hood);
    }
    @Override
    public boolean isFinished()
    {
        return robot_shooter.atSpeed();
    }

}
