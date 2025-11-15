package org.firstinspires.ftc.teamcode.autos;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;


import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "TestAutonew", group = "Examplenew")
public class TestOp extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int PathState;
    private Launcher launcher;

    private static final double velocity = 1800;

    private final Pose startPose = new Pose(88,8,Math.toRadians(90));
    private final Pose controlPoint1 = new Pose(120,48);
    private final Pose endPose = new Pose(84,84, Math.toRadians(45));

    private PathChain move;

    public void buildPaths(){

        move = follower
                .pathBuilder()
                .addPath(new BezierLine(new Pose(88.000, 8.000), new Pose(84.000, 84.000)))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(45))
                .build();
    }

    public void updates(){
        switch (PathState) {
            case 0:
                launcher.preSpin(velocity);
                follower.followPath(move);
                launcher.launch(velocity,true);
                setPathState(-1);
                break;
        }
    }

    public void setPathState(int i){
        PathState = i;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        updates();
        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", PathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void init() {
        launcher = new Launcher(hardwareMap);
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void stop() {}

}
