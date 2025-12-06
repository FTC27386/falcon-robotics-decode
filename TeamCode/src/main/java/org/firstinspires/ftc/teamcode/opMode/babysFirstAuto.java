package org.firstinspires.ftc.teamcode.opMode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.teamcode.Mechanisms.Commands.followPath;
import org.firstinspires.ftc.teamcode.Mechanisms.Commands.oneShot;
import org.firstinspires.ftc.teamcode.Mechanisms.Commands.runIntake;
import org.firstinspires.ftc.teamcode.Mechanisms.Commands.stopIntake;
import org.firstinspires.ftc.teamcode.Mechanisms.Paths;
import org.firstinspires.ftc.teamcode.Mechanisms.Robot;
import org.firstinspires.ftc.teamcode.Utility.RobotConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

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
        follower.update();
        PathChain pathChain = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(0, 0, Math.toRadians(0)),
                        new Pose(16, 28, Math.toRadians(90)))
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                .build();
        paths = new Paths(follower);
        register(r.getS(), r.getI());
        schedule(
                new SequentialCommandGroup(
             //   new oneShot(r),
                new runIntake(r),
                new followPath(r, paths.Path1), //intake 1st line
                new stopIntake(r),
                new followPath(r, paths.Path2), //return to shoot point
                //new oneShot(r),
                new followPath(r, paths.Path3),
                new runIntake(r),
                new followPath(r, paths.Path4),
                new stopIntake(r),
                new followPath(r, paths.Path5),
               // new oneShot(r),
                new followPath(r, paths.Path6),
                new runIntake(r),
                new followPath(r, paths.Path7),
                new stopIntake(r),
                new followPath(r, paths.Path8),
              //  new oneShot(r),
                new followPath(r, paths.Path9)));
    }
    @Override
    public void run()
    {
        super.run();
        telemetry.update();
    }
}
