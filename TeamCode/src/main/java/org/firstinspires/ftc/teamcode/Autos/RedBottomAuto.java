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

import org.firstinspires.ftc.teamcode.paths.Paths;
import kotlinx.coroutines.Delay;


/*
 *
 * START OP MODE RIGHT HALF (RED SIDE)
 * ON BOTTOM
 * ON TAPE
 * FACING TOWARDS OPPOSITE WALL
 *
 * */
@Autonomous(name = "redbottomauto", group = "redbottomauto")
public class RedBottomAuto extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private Intake nom;
    private Launcher pew;
    private int pathState;

    Paths paths = new Paths(follower);


    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(paths.Path1);
                setPathState(1);
                pew.launch(1750,true);
                break;
            case 1:
                if(!follower.isBusy()) {
                    follower.followPath(paths.Path2);
                    nom.intakeIn();
                    setPathState(2);
                }
                    break;
            case 2:
                if(!follower.isBusy()) {
                    follower.followPath(paths.Path3);
                    nom.stop();
                    setPathState(3);
                }
                    break;
            case 3:
                if(!follower.isBusy()) {
                    follower.followPath(paths.Path4);
                    pew.launch(1750,true);
                    setPathState(4);
                }
                    break;
            case 4:
                if(!follower.isBusy()) {
                    setPathState(-1);
                }
                break;
        }
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
        follower.setStartingPose(paths.startRedBottom);

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