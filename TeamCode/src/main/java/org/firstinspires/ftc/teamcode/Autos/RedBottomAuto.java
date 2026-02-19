package org.firstinspires.ftc.teamcode.Autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.paths.RedPathsFar;

@Autonomous(name = "redbottomauto", group = "redbottomauto")
public class RedBottomAuto extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    // Subsystems
    private Intake nom;
    private Launcher pew;

    // Paths
    private RedPathsFar paths;

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // --- ACTION: Start Path 1 (Move to Shoot) ---
                follower.followPath(paths.Path1);

                // Spin up immediately so we are ready upon arrival
                pew.preSpin(1750);

                setPathState(1);
                break;

            case 1:
                // --- ACTION: Wait for Path 1 Completion ---
                if(!follower.isBusy()) {
                    // We have arrived at the shooting position.
                    // Fire!
                    pew.launch(1750, true);

                    // Wait for the shot to finish
                    setPathState(2);
                }
                break;

            case 2:
                // --- ACTION: Wait for Shot to Finish ---
                if(!pew.isBusy()) {
                    // Shot is done.
                    // Start Path 2 (Move to Intake)
                    follower.followPath(paths.Path2);

                    // Turn on Intake
                    nom.intakeIn();

                    setPathState(3);
                }
                break;

            case 3:
                // --- ACTION: Wait for Path 2 (Intake Run) ---
                if(!follower.isBusy()) {
                    // Path 2 finished (Robot is at stack/intake pos).
                    // Move to Path 3
                    follower.followPath(paths.Path3);

                    // Stop Intake (or keep it running if Path 3 is short/transfer)
                    // Original code stopped it here:
                    nom.stop();

                    setPathState(4);
                }
                break;

            case 4:
                // --- ACTION: Wait for Path 3 ---
                if(!follower.isBusy()) {
                    // Path 3 finished.
                    // Start Path 4 (Return to Shooting Position)
                    follower.followPath(paths.Path4);

                    // Pre-spin again for the second shot
                    pew.preSpin(1750);

                    setPathState(5);
                }
                break;

            case 5:
                // --- ACTION: Wait for Path 4 (Arrival at Shoot) ---
                if(!follower.isBusy()) {
                    // Arrived. Fire second shot!
                    pew.launch(1750, true);
                    setPathState(6);
                }
                break;

            case 6:
                // --- ACTION: Wait for Shot 2 to Finish ---
                if(!pew.isBusy()) {
                    // Done. End Auto.
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
        // Update Subsystems
        pew.update();
        nom.update();

        // Update Follower
        follower.update();

        // Run Logic
        autonomousPathUpdate();

        // Telemetry
        telemetry.addData("Path State", pathState);
        telemetry.addData("Launcher Busy", pew.isBusy());

        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());

        // Flywheel Telemetry
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

        // Initialize Hardware
        pew = new Launcher(hardwareMap);
        nom = new Intake(hardwareMap);

        // Initialize Follower & Paths
        follower = Constants.createFollower(hardwareMap);
        paths = new RedPathsFar(follower);

        follower.setStartingPose(paths.startRedBottom);
        setPathState(0);
    }

    @Override
    public void init_loop() {}

    @Override
    public void stop() {}
}