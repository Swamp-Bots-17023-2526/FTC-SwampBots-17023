package org.firstinspires.ftc.teamcode.Autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "BlueBottom", group = "OpModes")
public class BlueBottomAuto extends OpMode {


    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private Intake intake;
    private Shooter Shooter;
    private Lift lift;
    private int pathState;

    private final Pose startRedClose = new Pose(118, 118, Math.toRadians(225));//right up against goal, facing away
    private final Pose startRedFar = new Pose(84, 12, Math.toRadians(90)); //on line, facing forward, middle of square
    private final Pose shootPoseFarRed = new Pose(84, 84, Math.toRadians(45)); //CHANGE ONCE VELOCITY DEFINED
    private final Pose shootPoseCloseRed = new Pose(84, 84, Math.toRadians(45)); //CHANGE ONCE VELOCITY DEFINED

    private final Pose startBlueClose = mirrorPose(startRedClose);
    private final Pose startBlueFar = mirrorPose(startRedFar);
    private final Pose shootPoseFarBlue = mirrorPose(shootPoseFarRed);
    private final Pose shootPoseCloseBlue = mirrorPose(shootPoseCloseRed);

    private final Pose RedObjects1 = new Pose(96, 84, Math.toRadians(180));
    private final Pose RedEndpickup1 = new Pose(115, 84, Math.toRadians(180));
    private final Pose RedObjects2 = new Pose(96, 60, Math.toRadians(180));
    private final Pose RedEndpickup2 = new Pose(115, 60, Math.toRadians(180));
    private final Pose RedObjects3 = new Pose(96, 36, Math.toRadians(180));
    private final Pose RedEndpickup3 = new Pose(115, 36, Math.toRadians(180));

    private final Pose BlueObjects1 = mirrorPose(RedObjects1);
    private final Pose BlueEndpickup1 = mirrorPose(RedEndpickup1);
    private final Pose BlueObjects2 = mirrorPose(RedObjects2);
    private final Pose BlueEndpickup2 = mirrorPose(RedEndpickup2);
    private final Pose BlueObjects3 = mirrorPose(RedObjects3);
    private final Pose BlueEndpickup3 = mirrorPose(RedEndpickup3);

    private PathChain moveToShoot, moveToObjects1, pickupObjects1, moveToShoot1, moveToObjects2, pickupObjects2, moveToShoot2, moveToObjects3, pickupObjects3, moveToShoot3;
    public void buildPaths() {

        moveToShoot = follower.pathBuilder()
                .addPath(new BezierLine(startRedFar, shootPoseFarRed))
                .setLinearHeadingInterpolation(startRedFar.getHeading(), shootPoseFarRed.getHeading())
                .build();

        moveToObjects1 = follower.pathBuilder()
                .addPath(new BezierLine(shootPoseFarRed, BlueObjects1))
                .setLinearHeadingInterpolation(shootPoseFarRed.getHeading(), BlueObjects1.getHeading())
                .build();

        pickupObjects1 = follower.pathBuilder()
                .addPath(new BezierLine(BlueObjects1, BlueEndpickup1))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        moveToShoot1 = follower.pathBuilder()
                .addPath(new BezierLine(BlueEndpickup1, shootPoseFarRed))
                .setLinearHeadingInterpolation(BlueEndpickup1.getHeading(), shootPoseFarRed.getHeading())
                .build();

        moveToObjects2 = follower.
                pathBuilder().addPath(new BezierLine(shootPoseFarRed, BlueObjects2))
                .setLinearHeadingInterpolation(shootPoseFarRed.getHeading(), BlueObjects2.getHeading())
                .build();

        pickupObjects2 = follower.pathBuilder()
                .addPath(new BezierLine(BlueObjects2, BlueEndpickup2))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        moveToShoot2 = follower.pathBuilder()
                .addPath(new BezierLine(BlueEndpickup2, shootPoseFarRed))
                .setLinearHeadingInterpolation(BlueEndpickup2.getHeading(), shootPoseFarRed.getHeading())
                .build();

        moveToObjects3 = follower.pathBuilder()
                .addPath(new BezierLine(shootPoseFarRed, BlueObjects3))
                .setLinearHeadingInterpolation(shootPoseFarRed.getHeading(), BlueObjects3.getHeading())
                .build();

        pickupObjects3 = follower.pathBuilder()
                .addPath(new BezierLine(BlueObjects3, RedEndpickup3))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        moveToShoot3 = follower.pathBuilder()
                .addPath(new BezierLine(RedEndpickup3, shootPoseFarRed))
                .setLinearHeadingInterpolation(RedEndpickup3.getHeading(), shootPoseFarRed.getHeading())
                .build();

    }


    public Pose mirrorPose(Pose input) {
        double inputX = input.getX();
        double inputY = input.getY();
        double inputHeading = input.getHeading();

        return new Pose(inputX - 72, inputY, 180 - inputHeading);
    }
}
