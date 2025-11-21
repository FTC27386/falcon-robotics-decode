package org.firstinspires.ftc.teamcode.Mechanisms;

import com.acmerobotics.dashboard.canvas.Canvas;

import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.geometry.Vector2d;

public final class Drawing {
    private Drawing() {}


    public static void drawRobot(Canvas c, Pose2d t) {
        final double ROBOT_RADIUS = 9;

        c.setStrokeWidth(1);
        c.strokeCircle(t.getX(), t.getY(), ROBOT_RADIUS);

    }
}