package org.firstinspires.ftc.teamcode.Mechanisms.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Mechanisms.Robot;

public class idleIntake extends CommandBase {

    private final Robot robot;

    public idleIntake(Robot robot) {
        this.robot = robot;
        addRequirements(robot.getI());
    }

    @Override
    public void initialize() {
        robot.getI().idleIntake();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
