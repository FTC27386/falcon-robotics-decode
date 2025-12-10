package org.firstinspires.ftc.teamcode.opMode;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Mechanisms.Commands.followPath;
import org.firstinspires.ftc.teamcode.Mechanisms.Commands.magDump;
import org.firstinspires.ftc.teamcode.Mechanisms.Commands.runIntake;
import org.firstinspires.ftc.teamcode.Mechanisms.Commands.stopIntake;
import org.firstinspires.ftc.teamcode.Mechanisms.Paths;
import org.firstinspires.ftc.teamcode.Mechanisms.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name="babysfirstauto")
public class babysFirstAuto extends CommandOpMode {
    Follower follower;
    private Robot r;
    Paths paths;
public static double speed_value = -2000;
public static double hood_angle = 0.5;

    @Override
    public void initialize()
    {
        super.reset();

        r = new Robot(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(Paths.startingPose);
        follower.update();
        paths = new Paths(follower);
        register(r.getS(), r.getI());
        schedule(new RunCommand(()->r.getS().setTurretPosition(r.getD().yoCalcAim())));
        schedule(
                new SequentialCommandGroup(
                        new InstantCommand(()-> r.setShooterValues()),
                        new InstantCommand(()-> r.getI().close()),

                        new followPath(r, paths.Path0),
               new magDump(r),
                new runIntake(r),
                        new InstantCommand(()-> r.setShooterValues()),
                new followPath(r, paths.Path1), //intake 1st line
                new stopIntake(r),
                new followPath(r, paths.Path2), //return to shoot point
                new magDump(r),
                new followPath(r, paths.Path3),
               new runIntake(r),
                        new InstantCommand(()-> r.setShooterValues()),
                new followPath(r, paths.Path4),
              new stopIntake(r),
                new followPath(r, paths.Path5),
                new magDump(r),
                new followPath(r, paths.Path6),
               new runIntake(r),
                        new InstantCommand(()-> r.setShooterValues()),
                new followPath(r, paths.Path7),
               new stopIntake(r),
                new followPath(r, paths.Path8),
              new magDump(r),
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
