package org.firstinspires.ftc.teamcode.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class BlueClosePath {
    public PathChain Path1;

    // Mirrored Start Pose:
    // X = 144 - 120.2 = 23.8
    // Y = 127 (Same)
    // Angle = 180 - 36 = 144 degrees
    public final Pose startBlueTop = new Pose(23.8, 127, Math.toRadians(144));

    public BlueClosePath(Follower follower) {
        Path1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(23.8, 127),

                                // Mirrored End Pose: X = 144 - 90 = 54
                                new Pose(54, 92)
                        )
                )
                // Mirrored Heading Interpolation: 180 - 36 = 144, 180 - 50 = 130
                .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(130))
                .build();
    }
}