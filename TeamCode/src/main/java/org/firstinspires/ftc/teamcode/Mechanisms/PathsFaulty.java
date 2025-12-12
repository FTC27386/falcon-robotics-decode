package org.firstinspires.ftc.teamcode.Mechanisms;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class PathsFaulty {

    public static Pose startingPose = new Pose(64.1883,132.5852,-Math.PI/2);
    public PathChain Path0;
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

    public PathsFaulty(Follower follower) {
        Path0 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(0, 144), new Pose(59.1, 79.0))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        Path1 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(59.100, 79.000),
                                new Pose(49.349, 85.797),
                                new Pose(16.000, 84.000)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        Path2 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(16.000, 84.000),
                                new Pose(48.958, 85.407),
                                new Pose(59.244, 79.027)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        Path3 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(59.244, 79.027), new Pose(44.000, 56))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        Path4 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(44.000, 56), new Pose(9.500, 56))
                )
                .setTangentHeadingInterpolation()
                .build();

        Path5 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(9.5, 56),
                                new Pose(47.300, 53),
                                new Pose(59.100, 79.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        Path6 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(59.100, 79.000), new Pose(44.000, 36.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        Path7 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(44.000, 36.000), new Pose(9.5, 36.000))
                )
                .setTangentHeadingInterpolation()
                .build();

        Path8 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(9.5, 36.000),
                                new Pose(52.845, 36.228),
                                new Pose(36.201, 55.993),
                                new Pose(59.100, 79.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        Path9 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(59.100, 79.000), new Pose(29.960, 71.181))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        park= follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(12, 12, Math.toRadians(90)), new Pose(39, 39, Math.toRadians(90)))
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

    }
}