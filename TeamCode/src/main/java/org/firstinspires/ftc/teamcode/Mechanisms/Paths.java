package org.firstinspires.ftc.teamcode.Mechanisms;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;


public class Paths {

    public PathChain Path1;
    public PathChain Path2;
    public PathChain Path3;
    public PathChain Path4;
    public PathChain Path5;
    public PathChain Path6;
    public PathChain Path7;
    public PathChain Path8;
    public PathChain Path9;
    public static Pose startingPose = new Pose(44.5, 83.8);

    public Paths(Follower follower) {
        Path1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(44.523, 83.871), new Pose(14.980, 84.080))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        Path2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(14.980, 84.080), new Pose(51.180, 83.871))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(315))
                .setReversed()
                .build();

        Path3 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(51.180, 83.871),
                                new Pose(66.576, 74.509),
                                new Pose(49.516, 59.946),
                                new Pose(41.069, 59.953)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        Path4 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(36.617, 59.946), new Pose(8.530, 59.946))
                )
                .setTangentHeadingInterpolation()
                .build();

        Path5 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(8.530, 59.946),
                                new Pose(47.020, 59.946),
                                new Pose(58.878, 77.214)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        Path6 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(58.878, 77.214),
                                new Pose(38.073, 55.369),
                                new Pose(63.663, 36.852),
                                new Pose(40.273, 35.758)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        Path7 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(36.201, 35.604), new Pose(8.739, 35.812))
                )
                .setTangentHeadingInterpolation()
                .build();

        Path8 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(8.739, 35.812),
                                new Pose(52.845, 36.228),
                                new Pose(36.201, 55.993),
                                new Pose(59.086, 77.214)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();
        Path9 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(59.086, 77.214), new Pose(29.960, 71.181))
                )
                .setLinearHeadingInterpolation(Math.toRadians(225), Math.toRadians(180))
                .build();
    }
}
