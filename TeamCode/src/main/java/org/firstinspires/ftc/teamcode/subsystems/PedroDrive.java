package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * Mecanum drive subsystem backed by PedroPathing.
 * - Field-centric teleop drive
 * - Face any point on the field
 * - Drive to a configured parking pose
 */
public class PedroDrive {

    private final Follower follower;

    // Whether Pedro is currently running an automated path/turn
    private boolean automatedDrive = false;

    // Default starting pose (you can change this)
    private Pose startingPose = new Pose(0, 0, 0);

    // Parking cube pose – YOU set these to your real field coordinates
    // Pedro coordinates: x right, y up, heading in radians.
    private Pose parkingPose = new Pose(60, 60, 0);   // placeholder

    public PedroDrive(HardwareMap hardwareMap) {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);
        follower.update();
    }

    /** Call this from your OpMode.start() */
    public void startTeleOp() {
        follower.startTeleopDrive();   // recommended by Pedro docs
    }

    /**
     * Field-centric drive.
     *
     * @param lx  left stick x  (strafe)
     * @param ly  left stick y  (forward/back)
     * @param rx  right stick x (rotation)
     */
    public void driveFieldCentric(double lx, double ly, double rx) {
        if (automatedDrive) return; // ignore manual commands during auto

        // Pedro docs: last param = true for robot-centric, false for field-centric.
        follower.setTeleOpDrive(
                -ly,        // forward/back
                -lx,        // strafe
                -rx,        // rotation
                false       // FIELD-CENTRIC
        );
    }

    /**
     * Turn in place so the robot faces the given (x, y) point on the field.
     * Uses follower.turnTo(targetHeading).
     */
    public void facePoint(double targetX, double targetY) {
        Pose pose = follower.getPose();

        double dx = targetX - pose.getX();
        double dy = targetY - pose.getY();

        // Pedro uses standard math: 0 rad facing +x, CCW positive.
        double targetHeading = Math.atan2(dy, dx);

        follower.turnTo(targetHeading);   // provided by Pedro’s Follower (used via SolversLib’s TurnToCommand docs).
        automatedDrive = true;
    }

    /**
     * Drive from current pose to the configured parkingPose using a simple
     * Bezier line and linear heading interpolation.
     */
    public void driveToParking() {
        PathChain parkingPath = buildPathTo(parkingPose);
        follower.followPath(parkingPath);
        automatedDrive = true;
    }

    /** Change the parking cube location at runtime. */
    public void setParkingPose(double x, double y, double headingRadians) {
        this.parkingPose = new Pose(x, y, headingRadians);
    }

    /** Optionally change starting pose (e.g., different alliance side). */
    public void setStartingPose(double x, double y, double headingRadians) {
        this.startingPose = new Pose(x, y, headingRadians);
        follower.setStartingPose(startingPose);
    }

    /** Must be called every loop from your OpMode. */
    public void update(boolean cancelAutomated) {
        follower.update();

        if (automatedDrive && (cancelAutomated || !follower.isBusy())) {
            // Return control back to teleop drive
            follower.startTeleopDrive();
            automatedDrive = false;
        }
    }

    public Pose getPose() {
        return follower.getPose();
    }

    public boolean isAutomated() {
        return automatedDrive;
    }

    // ---------- internal helpers ----------

    private PathChain buildPathTo(Pose target) {
        // Example taken from Pedro’s ExampleTeleOp format.
        return follower.pathBuilder()
                .addPath(new Path(
                        new BezierLine(follower::getPose, target)
                ))
                .setHeadingInterpolation(
                        HeadingInterpolator.linearFromPoint(
                                follower::getHeading,
                                target.getHeading(),
                                0.8   // how aggressively to turn along path
                        )
                )
                .build();
    }
}
