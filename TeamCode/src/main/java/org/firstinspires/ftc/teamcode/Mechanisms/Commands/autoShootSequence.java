package org.firstinspires.ftc.teamcode.Mechanisms.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Mechanisms.Robot;

//This is a command
public class autoShootSequence extends CommandBase {

    private final Robot robot;


    public autoShootSequence(Robot robot) {
        this.robot = robot;
        addRequirements(robot.getS());
    }

    @Override
    public void initialize() {
        robot.setAutoValues();
    }

    @Override
    public boolean isFinished() {
        return robot.getS().atSpeed();
    }

}
