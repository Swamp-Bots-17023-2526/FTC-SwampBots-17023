package org.firstinspires.ftc.teamcode.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class RedClosePath {
    public PathChain Path1;

    public final Pose startRedTop = new Pose(120.2,127, Math.toRadians(36));


    public RedClosePath(Follower follower) {
        Path1 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(120.2, 127),

                        new Pose(90,92)
                )
        ).setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(50)).build();
    }
}
