package org.firstinspires.ftc.teamcode.Autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.paths.BluePaths; // Import the Blue Paths

@Autonomous(name = "bluebottomauto", group = "bluebottomauto")
public class BlueBottomAuto extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private Intake nom;
    private Launcher pew;
    private int pathState;
    private BluePaths paths; // Use BluePaths class

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // --- ACTION: Start Path & Manual Wheel Feed ---
                follower.followPath(paths.Path1);

                // Run wheel forward briefly to seat the ring or unjam
                pew.manualWheelForward();

                actionTimer.resetTimer();
                setPathState(1);
                break;

            case 1:
                // --- ACTION: Wait for Wheel, Then Stop Wheel & Pre-Spin ---

                // Wait 0.5 seconds for the manual wheel to do its job
                if(actionTimer.getElapsedTimeSeconds() > 0.5) {
                    pew.manualWheelOff(); // Stop the wheel so it doesn't feed while spinning up
                    pew.preSpin(1750);    // Now start the flywheels

                    actionTimer.resetTimer(); // Reset timer for the spin-up wait
                    setPathState(2);
                }
                break;

            case 2:
                // --- ACTION: Wait for Spin Up, Then Fire ---

                // Wait 1.5 seconds for flywheels to reach speed
                if(actionTimer.getElapsedTimeSeconds() > 1.5) {
                    pew.launch(1750, true); // Fire!
                    setPathState(3);
                }
                break;

            case 3:
                // --- ACTION: Wait for Path 1 to Finish ---
                if(!follower.isBusy()) {
                    follower.followPath(paths.Path2);
                    nom.intakeIn();
                    setPathState(4);
                }
                break;

            case 4:
                // --- ACTION: Wait for Path 2 to Finish ---
                if(!follower.isBusy()) {
                    follower.followPath(paths.Path3);
                    nom.stop();
                    setPathState(5);
                }
                break;

            case 5:
                // --- ACTION: Wait for Path 3 to Finish ---
                if(!follower.isBusy()) {
                    follower.followPath(paths.Path4);

                    // Shoot immediately upon arrival
                    pew.launch(1750, true);
                    setPathState(6);
                }
                break;

            case 6:
                // --- ACTION: End ---
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

    @Override
    public void loop() {
        // Update subsystems every loop
        pew.update();
        nom.update();

        follower.update();
        autonomousPathUpdate();

        telemetry.addData("Path State", pathState);
        telemetry.addData("Action Timer", actionTimer.getElapsedTimeSeconds());
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
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
        paths = new BluePaths(follower); // Initialize BluePaths

        follower.setStartingPose(paths.startBlueBottom); // Use Blue Start Pose
        setPathState(0);
    }

    @Override
    public void init_loop() {}

    @Override
    public void stop() {}
}