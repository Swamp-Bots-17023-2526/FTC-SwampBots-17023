package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.subsystems.PedroDrive;
import org.firstinspires.ftc.teamcode.subsystems.test.VisionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.test.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.test.ShooterMath;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class MainTeleOp extends OpMode {

    private PedroDrive drive;
    private VisionSubsystem vision;
    private ShooterSubsystem shooter;

    // ID of the tag you want to aim at (e.g. 1 is Blue Left)
    private static final int TARGET_TAG_ID = 1;

    // State variables
    private boolean isAutoAiming = false;
    private double targetVelocity = 0.0;

    @Override
    public void init() {
        drive = new PedroDrive(hardwareMap);
        vision = new VisionSubsystem();
        shooter = new ShooterSubsystem();

        vision.init(hardwareMap, telemetry);
        shooter.init(hardwareMap);

        // Optional: Set a starting pose if you know it
        // drive.setStartingPose(0, 0, 0);
    }

    @Override
    public void start() {
        drive.startTeleOp();
    }

    @Override
    public void loop() {
        // 1. Update Vision
        vision.update();
        AprilTagDetection tag = vision.getSpecificTag(TARGET_TAG_ID);

        // 2. Control Inputs
        double forward = -gamepad1.left_stick_y;
        double strafe  = -gamepad1.left_stick_x;
        double turn    = -gamepad1.right_stick_x;

        // Button to activate Auto-Aim
        boolean aimRequested = gamepad1.right_bumper;

        // 3. Logic: Auto-Aim vs Manual
        if (aimRequested && tag != null) {
            isAutoAiming = true;

            // --- A. MOVEMENT ---
            // Calculate where the tag is in the world
            Pose robotPose = drive.getPose();
            double[] tagWorldPos = ShooterMath.calcTagCoordinates(robotPose, tag);

            // Tell PedroDrive to lock onto that coordinate
            drive.enableFaceTarget(tagWorldPos[0], tagWorldPos[1]);

            // --- B. SHOOTER ---
            // Calculate and set velocity
            targetVelocity = ShooterMath.calcShooterVelocity(tag);
            shooter.setTargetVelocity(targetVelocity);

        } else {
            // Manual Mode
            isAutoAiming = false;
            drive.disableFaceTarget();

            // Manual shooter control (e.g., A button to shoot static shot)
            if(gamepad1.a) {
                targetVelocity = 1500; // Fixed speed
            } else {
                targetVelocity = 0;
            }
            shooter.setTargetVelocity(targetVelocity);
        }

        // 4. Drive
        // Note: When FaceTarget is active, 'turn' (rx) is ignored by PedroDrive internally
        drive.driveFieldCentric(strafe, forward, turn);

        // 5. Update Pedro Internal Logic
        drive.update(false);

        // 6. Telemetry
        telemetry.addData("Mode", isAutoAiming ? "AUTO LOCKED" : "MANUAL");
        telemetry.addData("Target Vel", "%.0f ticks/s", targetVelocity);
        telemetry.addData("Actual Vel", "%.0f ticks/s", shooter.getVelocity());

        if(tag != null) {
            telemetry.addData("Tag Range", "%.1f inches", tag.ftcPose.range / 2.54);
            if(isAutoAiming && shooter.isReady(targetVelocity)) {
                telemetry.addData("SHOOTER", "!!! READY TO FIRE !!!");
            }
        } else if (aimRequested) {
            telemetry.addData("Warning", "No Tag Detected!");
        }

        telemetry.update();
    }

    @Override
    public void stop() {
        vision.stop();
        shooter.stop();
    }
}