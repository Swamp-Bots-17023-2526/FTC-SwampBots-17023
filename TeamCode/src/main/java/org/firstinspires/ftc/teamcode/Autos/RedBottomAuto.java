package org.firstinspires.ftc.teamcode.Autos;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import kotlinx.coroutines.Delay;


/*
 *
 * START OP MODE RIGHT HALF (RED SIDE)
 * ON BOTTOM
 * ON TAPE
 * FACING TOWARDS OPPOSITE WALL
 *
 * */
@Autonomous(name = "TestAuto", group = "Examples")
public class RedBottomAuto extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private Intake nom;
    private Launcher pew;
    private Lift uppies;
    private int pathState;

    private final Pose startRedClose = new Pose(118, 118, Math.toRadians(225));//right up against goal, facing away
    private final Pose startRedFar = new Pose(84, 12, Math.toRadians(90)); //on line, facing forward, middle of square
    private final Pose shootPoseFarRed = new Pose(84, 84, Math.toRadians(45)); //CHANGE ONCE VELOCITY DEFINED
    private final Pose shootPoseCloseRed = new Pose(84, 84, Math.toRadians(45)); //CHANGE ONCE VELOCITY DEFINED

    //object poses top to bottom
    private final Pose RedObjects1 = new Pose(96, 84, Math.toRadians(180));
    private final Pose RedEndpickup1 = new Pose(115, 84, Math.toRadians(180));
    private final Pose RedObjects2 = new Pose(96, 60, Math.toRadians(180));
    private final Pose RedEndpickup2 = new Pose(115, 60, Math.toRadians(180));
    private final Pose RedObjects3 = new Pose(96, 36, Math.toRadians(180));
    private final Pose RedEndpickup3 = new Pose(115, 36, Math.toRadians(180));

    private PathChain moveToShoot, moveToObjects1, pickupObjects1, moveToShoot1, moveToObjects2, pickupObjects2, moveToShoot2, moveToObjects3, pickupObjects3, moveToShoot3;


    public void buildPaths() {

        moveToShoot = follower.pathBuilder()
                .addPath(new BezierLine(startRedFar, shootPoseFarRed))
                .setLinearHeadingInterpolation(startRedFar.getHeading(), shootPoseFarRed.getHeading())
                .build();

        moveToObjects1 = follower.pathBuilder()
                .addPath(new BezierLine(shootPoseFarRed, RedObjects1))
                .setLinearHeadingInterpolation(shootPoseFarRed.getHeading(), RedObjects1.getHeading())
                .build();

        pickupObjects1 = follower.pathBuilder()
                .addPath(new BezierLine(RedObjects1, RedEndpickup1))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        moveToShoot1 = follower.pathBuilder()
                .addPath(new BezierLine(RedEndpickup1, shootPoseFarRed))
                .setLinearHeadingInterpolation(RedEndpickup1.getHeading(), shootPoseFarRed.getHeading())
                .build();

        moveToObjects2 = follower.
                pathBuilder().addPath(new BezierLine(shootPoseFarRed, RedObjects2))
                .setLinearHeadingInterpolation(shootPoseFarRed.getHeading(), RedObjects2.getHeading())
                .build();

        pickupObjects2 = follower.pathBuilder()
                .addPath(new BezierLine(RedObjects2, RedEndpickup2))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        moveToShoot2 = follower.pathBuilder()
                .addPath(new BezierLine(RedEndpickup2, shootPoseFarRed))
                .setLinearHeadingInterpolation(RedEndpickup2.getHeading(), shootPoseFarRed.getHeading())
                .build();

        moveToObjects3 = follower.pathBuilder()
                .addPath(new BezierLine(shootPoseFarRed, RedObjects3))
                .setLinearHeadingInterpolation(shootPoseFarRed.getHeading(), RedObjects3.getHeading())
                .build();

        pickupObjects3 = follower.pathBuilder()
                .addPath(new BezierLine(RedObjects3, RedEndpickup3))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        moveToShoot3 = follower.pathBuilder()
                .addPath(new BezierLine(RedEndpickup3, shootPoseFarRed))
                .setLinearHeadingInterpolation(RedEndpickup3.getHeading(), shootPoseFarRed.getHeading())
                .build();

    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(moveToShoot, true);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    //score preload
                    //have to make sure it scores both
                    pew.launch(1000, true);

                    follower.followPath(moveToObjects1, true);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    //start intake
                    nom.intakeIn();
                    follower.followPath(pickupObjects1, true);
                    setPathState(3);

                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    //stop intake then move to next pos
                    nom.stop();
                    follower.followPath(moveToShoot1, true);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    //shoot stuff
                    pew.launch(1000, true);
                    follower.followPath(moveToObjects2, true);
                    setPathState(5);

                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    //start intake
                    nom.intakeIn();
                    follower.followPath(pickupObjects2, true);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    nom.stop();
                    follower.followPath(moveToShoot2, true);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    //shoot stuff
                    pew.launch(1000, true);
                    follower.followPath(moveToObjects3, true);
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    //start intake
                    nom.intakeIn();
                    follower.followPath(pickupObjects3, true);
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    nom.stop();
                    //pause
                    follower.followPath(moveToShoot3, true);
                    setPathState(10);
                }
                break;
            case 10:
                if (!follower.isBusy()) {
                    //shoot stuff
                    pew.launch(1000, true);
                    setPathState(-1);
                }
            case 11:
                if (follower.isBusy()) {
                    //get a pos for parking
                    setPathState(-1);
                }
                break;

        }
    }

    public Pose mirrorPose(Pose input) {
        double inputX = input.getX();
        double inputY = input.getY();
        double inputHeading = input.getHeading();

        return new Pose(inputX - 72, inputY, 180 - inputHeading);
    }

    public void setPathState(int set) {
        pathState = set;
        pathTimer.resetTimer();
    }

    /**
     * This is the main loop of the OpMode, it will run repeatedly after clicking "Play".
     **/
    @Override
    public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /**
     * This method is called once at the init of the OpMode.
     **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();


        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startRedFar);

        opmodeTimer.resetTimer();
        setPathState(0);

    }

    /** This method is called continuously after Init while waiting for "play". **/
  /*  @Override
    public void periodic() {

    }
*/

    /**
     * This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system
     **/
    @Override
    public void init_loop() {

    }

    /**
     * We do not use this because everything should automatically disable
     **/
    @Override
    public void stop() {
    }


}