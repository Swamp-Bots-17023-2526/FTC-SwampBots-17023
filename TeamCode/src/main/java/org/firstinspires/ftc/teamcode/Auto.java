package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.jetbrains.annotations.ApiStatus;


@Autonomous(name = "Barrington")
public class Auto extends OpMode {

    private int PathState;
    private final Pose start = new Pose(0,0,Math.toRadians(90));
    private final Pose end = new Pose(0,25, Math.toRadians(90));


    private PathChain move;

    public void buildPaths(){
        move = follower.pathBuilder()
                .addPath(new BezierLine(start,end))
                .setTangentHeadingInterpolation()
                .build();
    }

    public void update(){
        switch(PathState){
            case 0:
                follower.followPath(move);
                setPathState(1);
                break;
            case 1:
                setPathState(-1);
                break;
        }
    }
    public void setPathState(int i){
        PathState = i;
    }
    @Override
    public void loop(){
        follower.update();
        update();

        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();

    }

    @Override
    public void init(){
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(start);
        setPathState(0);
    }

    @Override
    public void init_loop(){}

    @Override
    public void stop(){}

}
