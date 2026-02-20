package org.firstinspires.ftc.teamcode.Autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.paths.RedClosePath;

@Autonomous(name = "Red Close Auto", group = "Red")
public class RedCloseAuto extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private Intake nom;
    private Launcher pew;
    private RedClosePath paths;

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // --- ACTION: Start Path 1 ---
                follower.followPath(paths.Path1);
                pew.preSpin(1600);
                setPathState(1);
                break;

            case 1:
                // --- ACTION: Wait for Arrival, Then Fire ---
                if(!follower.isBusy()) {
                    // Trigger the shot
                    pew.launch(1600, true);

                    // Move to "Wait for Shot" state
                    setPathState(2);
                }
                break;

            case 2:
                // --- ACTION: Wait for Shot to Finish ---
                if(!pew.isBusy()) {
                    // The shot is finished (Launcher returned to IDLE).
                    // Move on to the next path.
                    setPathState(3); // FIXED: Was -1, now goes to 3
                }
                break;

            case 3:
                // --- ACTION: Start Path 2 ---
                // We know the robot isn't moving because it stopped to shoot.
                follower.followPath(paths.Path2);
                setPathState(4); // FIXED: Advance state so we don't spam followPath()
                break;

            case 4:
                // --- ACTION: Wait for Path 2 to finish ---
                if(!follower.isBusy()) {
                    // Path 2 is done. We can now idle or add more states.
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
        telemetry.addData("Launcher Busy", pew.isBusy()); // Display status
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());

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
        paths = new RedClosePath(follower);
        follower.setStartingPose(paths.startRedTop);

        setPathState(0);
    }

    @Override
    public void init_loop() {}

    @Override
    public void stop() {}
}