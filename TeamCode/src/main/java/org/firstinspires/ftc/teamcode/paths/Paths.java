package org.firstinspires.ftc.teamcode.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class Paths {
    public PathChain Path1, Path2, Path3, Path4;

    public final Pose startRedBottom = new Pose(86.620,9.225,Math.toRadians(90));

    public Paths(Follower follower) {
        Path1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(86.620, 9.225),

                                new Pose(83.995, 83.242)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(45))

                .build();

        Path2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(83.995, 83.242),

                                new Pose(101.859, 83.502)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(180))

                .build();

        Path3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(101.859, 83.502),

                                new Pose(126.021, 83.443)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path4 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(126.021, 83.443),

                                new Pose(84.072, 83.448)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(45))

                .build();
    }
}
