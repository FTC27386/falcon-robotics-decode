package org.firstinspires.ftc.teamcode.Mechanisms.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Mechanisms.Robot;
import org.firstinspires.ftc.teamcode.Utility.RobotConstants;

//This is a command
public class readyShooter extends CommandBase{

    private final Robot robot;


    public readyShooter(Robot robot)
    {
        this.robot = robot;
        addRequirements(robot.getS());
    }
    @Override
    public void initialize()
    {
        robot.prepShooter();
    }
    @Override
    public boolean isFinished()
    {
        return robot.getS().atSpeed();
    }

}
