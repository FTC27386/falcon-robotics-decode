package org.firstinspires.ftc.teamcode.Mechanisms;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class Paths {

    public static Pose startingPose = new Pose(8,8,Math.toRadians(90));
    public PathChain Path1;
    public PathChain Path2;
    public PathChain Path3;
    public PathChain Path4;
    public PathChain Path5;
    public PathChain Path6;
    public PathChain Path7;
    public PathChain Path8;
    public PathChain Path9;
    public PathChain park;

    public Paths(Follower follower) {
        Path1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(44.500, 84.000), new Pose(14.000, 84.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        Path2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(14.000, 84.000), new Pose(51.180, 84.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(315))
                .setReversed()
                .build();

        Path3 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(51.180, 84.000),
                                new Pose(64.887, 70.128),
                                new Pose(49.516, 59.946),
                                new Pose(41.400, 60.000)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        Path4 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(41.400, 60.000), new Pose(8.500, 60.000))
                )
                .setTangentHeadingInterpolation()
                .build();

        Path5 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(8.500, 60.000),
                                new Pose(47.300, 60.000),
                                new Pose(59.100, 79.000)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        Path6 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(59.100, 79.000), new Pose(41.400, 36.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(225), Math.toRadians(180))
                .build();

        Path7 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(41.400, 36.000), new Pose(8.500, 36.000))
                )
                .setTangentHeadingInterpolation()
                .build();

        Path8 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(8.500, 36.000),
                                new Pose(52.845, 36.228),
                                new Pose(36.201, 55.993),
                                new Pose(59.100, 79.000)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        Path9 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(59.100, 79.000), new Pose(29.960, 71.181))
                )
                .setLinearHeadingInterpolation(Math.toRadians(225), Math.toRadians(180))
                .build();

        park= follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(12, 12, Math.toRadians(90)), new Pose(39, 39, Math.toRadians(90)))
                )
                .setTangentHeadingInterpolation()
                .build();

    }
}