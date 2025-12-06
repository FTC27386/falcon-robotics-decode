package org.firstinspires.ftc.teamcode.Mechanisms.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Mechanisms.Robot;
import org.firstinspires.ftc.teamcode.Utility.RobotConstants;

//This is a command
public class readyShooter extends CommandBase{

    private final Robot robot;
    RobotConstants.robotState state;


    public readyShooter(Robot robot, RobotConstants.robotState state)
    {
        this.robot = robot;
        this.state = state;
        addRequirements(robot.getS());
    }
    @Override
    public void initialize()
    {
        robot.prepShooter(state);
    }
    @Override
    public boolean isFinished()
    {
        return robot.getS().atSpeed();
    }

}
