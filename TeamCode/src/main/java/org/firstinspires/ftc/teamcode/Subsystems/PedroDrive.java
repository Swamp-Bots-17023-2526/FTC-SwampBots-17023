package org.firstinspires.ftc.teamcode.Subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class PedroDrive {

    private final Follower follower;

    // Automated path / parking in progress
    private boolean automatedDrive = false;

    // Face-target mode for teleop
    private boolean faceTargetMode = false;
    private double faceTargetX = 0.0;
    private double faceTargetY = 0.0;

    // Default poses – set these to what makes sense for you
    private Pose startingPose = new Pose(0, 0, 0);
    private Pose parkingPose  = new Pose(60, 60, 0); // placeholder

    public PedroDrive(HardwareMap hardwareMap) {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);
        follower.update();
    }

    /** Call from OpMode.start() */
    public void startTeleOp() {
        follower.startTeleopDrive();
    }

    /**
     * Field-centric drive.
     * If faceTargetMode is enabled, rotation is automatically controlled to face the target.
     */
    public void driveFieldCentric(double lx, double ly, double rx) {
        // Ignore manual drive if we are currently following a path to parking
        if (automatedDrive) return;

        double forward = -ly;
        double strafe  = -lx;
        double turn;

        if (faceTargetMode) {
            // Auto-aim: compute heading error to target
            Pose pose = follower.getPose();
            double dx = faceTargetX - pose.getX();
            double dy = faceTargetY - pose.getY();

            double desiredHeading = Math.atan2(dy, dx);
            double currentHeading = pose.getHeading();

            double error = wrapAngle(desiredHeading - currentHeading);

            // Simple proportional control on heading error
            double kP = 1.0; // tune this
            turn = clamp(kP * error, -1.0, 1.0);
        } else {
            // Normal driver rotation
            turn = -rx;
        }

        // false -> field-centric, using Pedro’s localization
        follower.setTeleOpDrive(forward, strafe, turn, false);
    }

    /** Enable "always face this field point" mode. */
    public void enableFaceTarget(double x, double y) {
        faceTargetMode = true;
        faceTargetX = x;
        faceTargetY = y;
    }

    /** Disable face-target mode. */
    public void disableFaceTarget() {
        faceTargetMode = false;
    }

    public boolean isFaceTargetMode() {
        return faceTargetMode;
    }

    /** Drive from current pose to parkingPose using Pedro path. */
    public void driveToParking() {
        PathChain parkingPath = buildPathTo(parkingPose);
        follower.followPath(parkingPath);
        automatedDrive = true;
    }

    public void setParkingPose(double x, double y, double headingRadians) {
        this.parkingPose = new Pose(x, y, headingRadians);
    }

    public void setStartingPose(double x, double y, double headingRadians) {
        this.startingPose = new Pose(x, y, headingRadians);
        follower.setStartingPose(startingPose);
    }

    /** Force-set the robot's pose estimate mid-teleop or anytime. */
    public void resetPose(double x, double y, double headingRadians) {
        follower.setPose(new Pose(x, y, headingRadians));
    }

    /** Call once per loop. cancelAutomated = true to abort auto path. */
    public void update(boolean cancelAutomated) {
        follower.update();

        if (automatedDrive && (cancelAutomated || !follower.isBusy())) {
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
        return follower.pathBuilder()
                .addPath(new Path(
                        new BezierLine(follower::getPose, target)
                ))
                .setHeadingInterpolation(
                        HeadingInterpolator.linearFromPoint(
                                follower::getHeading,
                                target.getHeading(),
                                0.8
                        )
                )
                .build();
    }

    private static double wrapAngle(double angle) {
        while (angle > Math.PI)  angle -= 2.0 * Math.PI;
        while (angle < -Math.PI) angle += 2.0 * Math.PI;
        return angle;
    }

    private static double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }
}