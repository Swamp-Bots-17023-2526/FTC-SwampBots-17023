package org.firstinspires.ftc.teamcode.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class BluePaths {
    public PathChain Path1, Path2, Path3, Path4;

    // Mirrored Start Pose: X = 144 - 86.620 = 57.38
    public final Pose startBlueBottom = new Pose(57.38, 9.225, Math.toRadians(90));

    public BluePaths(Follower follower) {
        // Path 1: Move to Shooting Position
        // End X = 144 - 83.995 = 60.005
        // Heading 45 -> 135
        Path1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(57.38, 9.225),
                                new Pose(60.005, 83.242)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(135))
                .build();

        // Path 2: Move to Stack/Intake Pos 1
        // End X = 144 - 101.859 = 42.141
        // Heading 180 -> 0
        Path2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(60.005, 83.242),
                                new Pose(42.141, 83.502)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(0))
                .build();

        // Path 3: Move deeper into Stack/Intake Pos 2
        // End X = 144 - 126.021 = 17.979
        // Heading Constant 0
        Path3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(42.141, 83.502),
                                new Pose(17.979, 83.443)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        // Path 4: Return to Shooting Position
        // End X = 144 - 84.072 = 59.928
        // Heading 0 -> 135
        Path4 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(17.979, 83.443),
                                new Pose(59.928, 83.448)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(135))
                .build();
    }
}