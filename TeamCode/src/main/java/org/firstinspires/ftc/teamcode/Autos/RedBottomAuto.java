package org.firstinspires.ftc.teamcode.autos;


import static org.firstinspires.ftc.teamcode.autos.Poses.*;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;




/*
 *
 * START OP MODE RIGHT HALF (RED SIDE)
 * ON BOTTOM
 * ON TAPE
 * FACING TOWARDS OPPOSITE WALL
 *
 * */
@Autonomous(name = "RedBottomAuto", group = "Examples")
public class RedBottomAuto extends OpMode {

    //public RedBottomAuto() {
    //  addComponents(new SubsystemComponent(Intake.INSTANCE, Flywheel.INSTANCE, Loader.INSTANCE, Lift.INSTANCE), BulkReadComponent.INSTANCE, BindingsComponent.INSTANCE);
    //}

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;

    private final Pose startRedClose = new Pose(118, 118, Math.toRadians(225));//right up against goal, facing away
    private final Pose startRedFar = new Pose(86, 9, Math.toRadians(90)); //on line, facing forward, middle of square
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
                //pause
                //flywheel shoot
                //new Delay(1);

                follower.followPath(moveToShoot, true);
                //for shooting 2 balls
//                new SequentialGroup(
//                        //flywheel shoot,
//                        Flywheel.INSTANCE.shoot_short,
//                        //stop the flys
//                        Flywheel.INSTANCE.stop,
//                        //load next ball
//                        Loader.INSTANCE.push,
//                        //reset the loader
//                        Loader.INSTANCE.reset,
//                        //Shoot second preload
//                        Flywheel.INSTANCE.shoot_short, Flywheel.INSTANCE.stop);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(moveToObjects1, true);
                    //start intake
                    //Intake.INSTANCE.spin();
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(pickupObjects1, true);
                    //stop intake
                    //Intake.INSTANCE.stop();
                    setPathState(3);
                }
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(moveToShoot1, true);
                    //pause
//                    new Delay(3);
//                    //flywheel shoot
//                    new SequentialGroup(
//                            //flywheel shoot,
//                            Flywheel.INSTANCE.shoot_short,
//                            //stop the flys
//                            Flywheel.INSTANCE.stop,
//                            //load next ball
//                            Loader.INSTANCE.push,
//                            //reset the loader
//                            Loader.INSTANCE.reset,
//                            //Shoot second preload
//                            Flywheel.INSTANCE.shoot_short, Flywheel.INSTANCE.stop);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {

                    follower.followPath(moveToObjects2, true);
                    //Intake.INSTANCE.spin();
                    //start intake
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(pickupObjects2, true);
                    //stop intake
                    setPathState(6);
                    //Intake.INSTANCE.stop();
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    //pause
                    follower.followPath(moveToShoot2, true);
//                    new Delay(.5);
//                    //flywheel shoot
//                    new SequentialGroup(
//                            //flywheel shoot,
//                            Flywheel.INSTANCE.shoot_short,
//                            //stop the flys
//                            Flywheel.INSTANCE.stop,
//                            //load next ball
//                            Loader.INSTANCE.push,
//                            //reset the loader
//                            Loader.INSTANCE.reset,
//                            //Shoot second preload
//                            Flywheel.INSTANCE.shoot_short,
//                            Flywheel.INSTANCE.stop
//                    );
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(moveToObjects3, true);
                    //start intake
                    //Intake.INSTANCE.spin();
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(pickupObjects3, true);
                    //stop intake
                    //Intake.INSTANCE.stop();
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    //pause
                    follower.followPath(moveToShoot3, true);
//                    new Delay(.5);
//                    //flywheel shoot
//                    new SequentialGroup(
//                            //flywheel shoot,
//                            Flywheel.INSTANCE.shoot_short,
//                            //stop the flys
//                            Flywheel.INSTANCE.stop,
//                            //load next ball
//                            Loader.INSTANCE.push,
//                            //reset the loader
//                            Loader.INSTANCE.reset,
//                            //Shoot second preload
//                            Flywheel.INSTANCE.shoot_short,
//                            Flywheel.INSTANCE.stop
//                    );
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
    public void init_loop() {}
    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }
    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {}

}
