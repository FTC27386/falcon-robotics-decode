package org.firstinspires.ftc.teamcode.opMode;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.teamcode.Mechanisms.Commands.oneShot;
import org.firstinspires.ftc.teamcode.Mechanisms.Commands.runIntake;
import org.firstinspires.ftc.teamcode.Mechanisms.Commands.stopIntake;
import org.firstinspires.ftc.teamcode.Mechanisms.Paths;
import org.firstinspires.ftc.teamcode.Mechanisms.Robot;
import org.firstinspires.ftc.teamcode.Utility.RobotConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

@Autonomous(name="babysfirstauto")
public class babysFirstAuto extends CommandOpMode {
    Follower follower;
    private Robot r;
    Paths paths;

    @Override
    public void initialize()
    {
        super.reset();

        r = new Robot(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(Paths.startingPose);
        paths = new Paths(follower);
        register(r.getS(), r.getI());
        schedule(new SequentialCommandGroup(
                new oneShot(r, RobotConstants.robotState.CLOSE_SHOOT),
                new runIntake(r),
                new FollowPathCommand(follower, paths.Path1), //intake 1st line
                new stopIntake(r),
                new FollowPathCommand(follower, paths.Path2), //return to shoot point
                new oneShot(r, RobotConstants.robotState.CLOSE_SHOOT),
               new FollowPathCommand(follower, paths.Path3),
                new runIntake(r),
               new FollowPathCommand(follower, paths.Path4),
                new stopIntake(r),
                new FollowPathCommand(follower, paths.Path5),
                new oneShot(r, RobotConstants.robotState.CLOSE_SHOOT),
                new FollowPathCommand(follower, paths.Path6),
                new runIntake(r),
                new FollowPathCommand(follower, paths.Path7),
                new stopIntake(r),
                new FollowPathCommand(follower, paths.Path8),
                new oneShot(r, RobotConstants.robotState.CLOSE_SHOOT),
                new FollowPathCommand(follower, paths.Path9)));

    }
    @Override
    public void run()
    {

        super.run();

    }
}
