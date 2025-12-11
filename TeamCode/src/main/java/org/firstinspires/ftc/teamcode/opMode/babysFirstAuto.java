package org.firstinspires.ftc.teamcode.opMode;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Mechanisms.Commands.followPath;
import org.firstinspires.ftc.teamcode.Mechanisms.Commands.autoShot;
import org.firstinspires.ftc.teamcode.Mechanisms.Commands.runIntake;
import org.firstinspires.ftc.teamcode.Mechanisms.Commands.stopIntake;
import org.firstinspires.ftc.teamcode.Mechanisms.PathsFaulty;
import org.firstinspires.ftc.teamcode.Mechanisms.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name="babysfirstauto")
public class babysFirstAuto extends CommandOpMode {
    Follower follower;
    private Robot r;
    PathsFaulty paths;
public static double speed_value = -1900;
public static double hood_angle = 0.8;

    @Override
    public void initialize()
    {
        super.reset();

        r = new Robot(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(PathsFaulty.startingPose);
        follower.update();
        paths = new PathsFaulty(follower);
        register(r.getS(), r.getI());

        schedule(
                new SequentialCommandGroup(
                        new InstantCommand(()->r.setShooterValues()),

                        new InstantCommand(()-> r.setShooterValues()),
                        new InstantCommand(()-> r.getI().close()),
                        new followPath(r, paths.Path0),
                         new autoShot(r),
                        new runIntake(r),
                        new InstantCommand(()-> r.setShooterValues()),
                new followPath(r, paths.Path1), //intake 1st line

                new ParallelCommandGroup(
                                new followPath(r,paths.Path2),
                                new SequentialCommandGroup(
                                        new WaitCommand(1000),
                                                new stopIntake(r))

                ),
                new autoShot(r),
                new followPath(r, paths.Path3),
               new runIntake(r),
                        new InstantCommand(()-> r.setShooterValues()),
                new followPath(r, paths.Path4),
                new ParallelCommandGroup(
                        new followPath(r,paths.Path5),
                        new SequentialCommandGroup(
                                new WaitCommand(1000),
                                new stopIntake(r))
                ),
                new autoShot(r),
                new followPath(r, paths.Path6),
               new runIntake(r),
                        new InstantCommand(()-> r.setShooterValues()),
                new followPath(r, paths.Path7),
                new ParallelCommandGroup(
                        new followPath(r,paths.Path8),
                        new SequentialCommandGroup(
                                new WaitCommand(1000),
                                new stopIntake(r))
                ),
              new autoShot(r),
                new followPath(r, paths.Path9)));
    }
    @Override
    public void run()
    {
        super.run();
        telemetry.addData("turretPose",r.getS().getTurretPosition());
        telemetry.addData("robot X", r.getD().getCurrentPose().getX());
        telemetry.addData("robot Y", r.getD().getCurrentPose().getY());
        telemetry.addData("robot heading", Math.toDegrees(r.getD().getCurrentPose().getHeading()) );
        telemetry.addData("target X",r.getD().getTarg().getX());
        telemetry.addData("target Y",r.getD().getTarg().getY());

        telemetry.update();
    }
}
