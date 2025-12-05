package org.firstinspires.ftc.teamcode.Autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "BlueBottom", group = "OpModes")
public class BlueBottomAuto extends OpMode {


    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private Intake intake;
    private Launcher Shooter;
    private Lift lift;
    private int pathState;

    //for red
    private final Pose startRedClose = new Pose(118, 118, Math.toRadians(225));//right up against goal, facing away
    private final Pose startRedFar = new Pose(84, 12, Math.toRadians(90)); //on line, facing forward, middle of square
    private final Pose shootPoseFarRed = new Pose(84, 84, Math.toRadians(45)); //CHANGE ONCE VELOCITY DEFINED
    private final Pose shootPoseCloseRed = new Pose(84, 84, Math.toRadians(45)); //CHANGE ONCE VELOCITY DEFINED

    //for blue
    private final Pose startBlueClose = mirrorPose(startRedClose);
    private final Pose startBlueFar = mirrorPose(startRedFar);
    private final Pose shootPoseFarBlue = mirrorPose(shootPoseFarRed);
    private final Pose shootPoseCloseBlue = mirrorPose(shootPoseCloseRed);


    //for red
    private final Pose RedObjects1 = new Pose(96, 84, Math.toRadians(180));
    private final Pose RedEndpickup1 = new Pose(115, 84, Math.toRadians(180));
    private final Pose RedObjects2 = new Pose(96, 60, Math.toRadians(180));
    private final Pose RedEndpickup2 = new Pose(115, 60, Math.toRadians(180));
    private final Pose RedObjects3 = new Pose(96, 36, Math.toRadians(180));
    private final Pose RedEndpickup3 = new Pose(115, 36, Math.toRadians(180));

    //for blue
    private final Pose BlueObjects1 = mirrorPose(RedObjects1);
    private final Pose BlueEndpickup1 = mirrorPose(RedEndpickup1);
    private final Pose BlueObjects2 = mirrorPose(RedObjects2);
    private final Pose BlueEndpickup2 = mirrorPose(RedEndpickup2);
    private final Pose BlueObjects3 = mirrorPose(RedObjects3);
    private final Pose BlueEndpickup3 = mirrorPose(RedEndpickup3);

    private PathChain moveToShoot, moveToObjects1, pickupObjects1, moveToShoot1, moveToObjects2, pickupObjects2, moveToShoot2, moveToObjects3, pickupObjects3, moveToShoot3;
    public void buildPaths() {

        moveToShoot = follower.pathBuilder()
                .addPath(new BezierLine(startBlueFar, shootPoseFarBlue))
                .setLinearHeadingInterpolation(startBlueFar.getHeading(), shootPoseFarBlue.getHeading())
                .build();

        moveToObjects1 = follower.pathBuilder()
                .addPath(new BezierLine(shootPoseFarBlue, BlueObjects1))
                .setLinearHeadingInterpolation(shootPoseFarBlue.getHeading(), BlueObjects1.getHeading())
                .build();

        pickupObjects1 = follower.pathBuilder()
                .addPath(new BezierLine(BlueObjects1, BlueEndpickup1))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        moveToShoot1 = follower.pathBuilder()
                .addPath(new BezierLine(BlueEndpickup1, shootPoseFarBlue))
                .setLinearHeadingInterpolation(BlueEndpickup1.getHeading(), shootPoseFarBlue.getHeading())
                .build();

        moveToObjects2 = follower.
                pathBuilder().addPath(new BezierLine(shootPoseFarBlue, BlueObjects2))
                .setLinearHeadingInterpolation(shootPoseFarBlue.getHeading(), BlueObjects2.getHeading())
                .build();

        pickupObjects2 = follower.pathBuilder()
                .addPath(new BezierLine(BlueObjects2, BlueEndpickup2))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        moveToShoot2 = follower.pathBuilder()
                .addPath(new BezierLine(BlueEndpickup2, shootPoseFarBlue))
                .setLinearHeadingInterpolation(BlueEndpickup2.getHeading(), shootPoseFarBlue.getHeading())
                .build();

        moveToObjects3 = follower.pathBuilder()
                .addPath(new BezierLine(shootPoseFarBlue, BlueObjects3))
                .setLinearHeadingInterpolation(shootPoseFarBlue.getHeading(), BlueObjects3.getHeading())
                .build();

        pickupObjects3 = follower.pathBuilder()
                .addPath(new BezierLine(BlueObjects3, BlueEndpickup3))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        moveToShoot3 = follower.pathBuilder()
                .addPath(new BezierLine(BlueEndpickup3, shootPoseFarBlue))
                .setLinearHeadingInterpolation(BlueEndpickup3.getHeading(), shootPoseFarBlue.getHeading())
                .build();

    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                //pause
                //flywheel shoot
                follower.followPath(moveToShoot, true);
                //for shooting 2 balls

                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(moveToObjects1, true);
                    //start intake
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(pickupObjects1, true);
                    //stop intake
                    setPathState(3);
                }
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(moveToShoot1, true);
                    //pause

                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {

                    follower.followPath(moveToObjects2, true);
                    //start intake
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(pickupObjects2, true);
                    //stop intake
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    //pause
                    follower.followPath(moveToShoot2, true);

                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(moveToObjects3, true);
                    //start intake
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(pickupObjects3, true);
                    //stop intake
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    //pause
                    follower.followPath(moveToShoot3, true);

                    setPathState(10);
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