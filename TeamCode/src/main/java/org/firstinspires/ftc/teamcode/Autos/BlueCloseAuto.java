package org.firstinspires.ftc.teamcode.Autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

// --- IMPORTANT IMPORT CHANGE ---
import org.firstinspires.ftc.teamcode.paths.BlueClosePath;

@Autonomous(name = "Blue Close Auto", group = "Blue")
public class BlueCloseAuto extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private Intake nom;
    private Launcher pew;

    // Use the Blue Path class
    private BlueClosePath paths;

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // --- ACTION: Start Path 1 ---
                follower.followPath(paths.Path1);
                pew.preSpin(1400);
                setPathState(1);
                break;

            case 1:
                // --- ACTION: Wait for Arrival, Then Fire ---
                if(!follower.isBusy()) {
                    // Trigger the shot
                    pew.launch(1400, true);

                    // Move to "Wait for Shot" state
                    setPathState(2);
                }
                break;

            case 2:
                // --- ACTION: Wait for Shot to Finish ---
                if(!pew.isBusy()) {
                    // Shot is done, launcher is IDLE
                    setPathState(-1);
                }
                break;

            case -1:
                // --- ACTION: End / Idle ---
                break;
        }
    }

    public void setPathState(int set) {
        pathState = set;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        pew.update();
        nom.update();
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("Path State", pathState);
        telemetry.addData("Launcher Busy", pew.isBusy());
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading (Deg)", Math.toDegrees(follower.getPose().getHeading()));

        telemetry.addData("Target Vel", pew.getTargetVelocity());
        telemetry.addData("Left Vel",   pew.getLeftVelocity());
        telemetry.addData("Right Vel",  pew.getRightVelocity());

        telemetry.update();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        pew = new Launcher(hardwareMap);
        nom = new Intake(hardwareMap);

        follower = Constants.createFollower(hardwareMap);

        // Initialize Blue Paths
        paths = new BlueClosePath(follower);

        // Set Starting Pose to Blue Top
        follower.setStartingPose(paths.startBlueTop);

        setPathState(0);
    }

    @Override
    public void init_loop() {}

    @Override
    public void stop() {}
}