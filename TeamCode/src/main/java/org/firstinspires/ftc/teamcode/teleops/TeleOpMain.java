package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.PedroDrive;

@TeleOp(name = "Full Robot TeleOp (Manual Only)", group = "Competition")
public class TeleOpMain extends LinearOpMode {

    // --- Subsystems ---
    private PedroDrive drive;
    private Launcher launcher;
    private Intake intake;

    // --- State Variables ---
    private boolean isRedAlliance = true;

    // --- Manual RPM Variable ---
    private double manualTargetRpm = 1700.0;

    // --- Toggle Memory ---
    private boolean lastRightBumper = false;
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastButtonX = false;
    private boolean lastBackButton = false;

    // --- Goal Coordinates (Still used for Aim Lock) ---
    private static final double RED_GOAL_X = 138;
    private static final double RED_GOAL_Y = 138;
    private static final double BLUE_GOAL_X = 4.5;
    private static final double BLUE_GOAL_Y = 139;

    @Override
    public void runOpMode() throws InterruptedException {
        // 1. Initialize Subsystems
        drive = new PedroDrive(hardwareMap);
        launcher = new Launcher(hardwareMap);
        intake = new Intake(hardwareMap);

        // 2. Alliance Selection Loop
        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.a) isRedAlliance = true;
            if (gamepad1.x) isRedAlliance = false;

            telemetry.addLine("=== MANUAL TELEOP INIT ===");
            telemetry.addData("Alliance", isRedAlliance ? "RED" : "BLUE");
            telemetry.addLine("Press: a for Red, x for Blue");
            telemetry.addLine("RPM Mode: MANUAL ONLY");
            telemetry.update();
        }

        // 3. Set Starting Pose based on Alliance
        if (isRedAlliance) {
            drive.setStartingPose(84.072, 83.448, Math.toRadians(45));
        } else {
            drive.setStartingPose(9, 120, Math.toRadians(0));
        }

        drive.startTeleOp();

        // 4. Main TeleOp Loop
        while (opModeIsActive()) {

            // --- 1. DRIVING & AIMING ---

            // Aim Lock (Right Bumper)
            if (gamepad1.right_bumper && !lastRightBumper) {
                if (drive.isFaceTargetMode()) {
                    drive.disableFaceTarget();
                } else {
                    if (isRedAlliance) drive.enableFaceTarget(RED_GOAL_X, RED_GOAL_Y);
                    else drive.enableFaceTarget(BLUE_GOAL_X, BLUE_GOAL_Y);
                }
            }
            lastRightBumper = gamepad1.right_bumper;

            // Reset Heading (Back Button) - Fixes Gyro Drift
            if (gamepad1.back && !lastBackButton) {
                Pose currentPose = drive.getPose();
                drive.resetPose(currentPose.getX(), currentPose.getY(), 0);
            }
            lastBackButton = gamepad1.back;

            // Drive Command
            // We pass 'false' to update() because there is no auto-path to cancel anymore
            drive.driveFieldCentric(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
            drive.update(false);


            // --- 2. INTAKE & FEEDER CONTROLS ---

            // Button X: Toggle Intake AND Manual Feeder Wheel
            if (gamepad1.x && !lastButtonX) {
                if (intake.getState() == Intake.State.INTAKING) {
                    // Turn OFF Intake & Wheel
                    intake.stop();
                    launcher.manualWheelOff();
                } else {
                    // Turn ON Intake & Wheel
                    intake.intakeIn();
                    launcher.manualWheelForward();
                }
            }
            lastButtonX = gamepad1.x;

            // Button Y: Hold to OUTTAKE (Reverse)
            if (gamepad1.y) {
                intake.intakeOut();
                launcher.manualWheelBack();
            } else if (!gamepad1.x && intake.getState() == Intake.State.OUTTAKING) {
                intake.stop();
                launcher.manualWheelOff();
            }


            // --- 3. LAUNCHER CONTROLS ---

            // Button A: Spin Up
            if (gamepad1.a) {
                launcher.preSpin(manualTargetRpm);
            }

            // Triggers: Fire
            if (gamepad1.right_trigger > 0.5) {
                launcher.launch(manualTargetRpm, false);
            } else if (gamepad1.left_trigger > 0.5) {
                launcher.launch(manualTargetRpm, true);
            }

            // Left Bumper: Advance Second Artifact
            if (gamepad1.left_bumper) {
                launcher.advanceSecondArtifact();
            }

            // Button B: STOP ALL (Safety)
            if (gamepad1.b) {
                launcher.stopAll();
                intake.stop();
            }


            // --- 4. ADJUSTMENTS (D-PAD) ---

            // D-Pad UP/DOWN: Adjust Manual RPM
            if (gamepad1.dpad_up && !lastDpadUp) {
                manualTargetRpm += 50;
            }
            lastDpadUp = gamepad1.dpad_up;

            if (gamepad1.dpad_down && !lastDpadDown) {
                manualTargetRpm -= 50;
            }
            lastDpadDown = gamepad1.dpad_down;

            // D-Pad LEFT/RIGHT: Manual Launcher Unjam
            if (gamepad1.dpad_left) {
                launcher.manualWheelForward();
            } else if (gamepad1.dpad_right) {
                launcher.manualWheelBack();
            } else {
                // Only stop the wheel if:
                // 1. We aren't intaking (because intake keeps it running now)
                // 2. We aren't firing/advancing (launcher logic)
                String lState = launcher.getStateName();
                if (intake.getState() != Intake.State.INTAKING &&
                        !lState.contains("FIRING") &&
                        !lState.contains("ADVANCING")) {
                    launcher.manualWheelOff();
                }
            }

            // Update Systems
            launcher.update();
            intake.update();

            // --- 5. TELEMETRY ---
            telemetry.addData("TARGET RPM", "%.0f", manualTargetRpm);
            telemetry.addData("Intake", intake.getState());
            telemetry.addData("Launcher L/R", "%.0f / %.0f", launcher.getLeftVelocity(), launcher.getRightVelocity());
            telemetry.update();
        }
    }
}