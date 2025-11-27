package org.firstinspires.ftc.teamcode.Mechanisms.Commands;

import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Mechanisms.Robot;

public class pathFollow extends CommandBase {

    Robot r;
    PathChain path;

    public pathFollow(Robot r, PathChain path)
    {
        this.r = r;
        this.path = path;
    }

    @Override
    public void initialize()
    {
        r.getD().follower.followPath(path);
    }
    @Override
    public boolean isFinished()
    {
        return r.getD().follower.atParametricEnd();
    }
}
