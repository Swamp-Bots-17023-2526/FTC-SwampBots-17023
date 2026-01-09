package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.PedroDrive;
import org.firstinspires.ftc.teamcode.subsystems.test.PedroCalculate;

@TeleOp(name = "Single Controller TeleOp (Switch)", group = "Competition")
public class TeleOpTest extends LinearOpMode {

    // --- Subsystems ---
    private PedroDrive drive;
    private Launcher launcher;
    private PedroCalculate calculator;

    // --- Enums for State Machine ---
    private enum RpmMode {
        AUTO_TARGETING, // Uses Position -> Math -> RPM
        MANUAL_FIXED    // Uses Driver set RPM
    }
    private RpmMode currentRpmMode = RpmMode.AUTO_TARGETING;

    // --- State Variables ---
    private boolean isRedAlliance = true;

    // Variables for calculations
    private double calculatedRpm = 0.0;
    private double manualBaseRpm = 3000.0; // Default starting RPM for manual mode
    private double rpmOffset = 0.0;        // Trim adjustment for AUTO mode
    private double finalTargetRpm = 0.0;

    // --- Toggle Memory ---
    private boolean lastRightBumper = false;
    private boolean lastStartButton = false;
    private boolean lastLeftStickButton = false; // For Mode Toggle
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;

    // --- Goal Coordinates ---
    private static final double RED_GOAL_X = 138;
    private static final double RED_GOAL_Y = 138;
    private static final double BLUE_GOAL_X = 4.5;
    private static final double BLUE_GOAL_Y = 139;

    @Override
    public void runOpMode() throws InterruptedException {
        // 1. Initialize
        drive = new PedroDrive(hardwareMap);
        launcher = new Launcher(hardwareMap);
        calculator = new PedroCalculate(hardwareMap);

        // 2. Alliance Selection
        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.a) isRedAlliance = true;
            if (gamepad1.x) isRedAlliance = false;

            telemetry.addLine("=== SINGLE CONTROLLER SETUP ===");
            telemetry.addData("Alliance", isRedAlliance ? "RED" : "BLUE");
            telemetry.addLine("L3 (Left Stick Click) -> Toggle Auto/Manual RPM");
            telemetry.update();
        }

        // 3. Setup Calculator
        if (isRedAlliance) calculator.setRed();
        else calculator.setBlue();
        calculator.setGoal();

        drive.startTeleOp();

        // 4. Main Loop
        while (opModeIsActive()) {

            // =========================================================
            // RPM STATE MACHINE (The Requested Switch Case)
            // =========================================================

            // Toggle Mode with Left Stick Button (L3)
            if (gamepad1.left_stick_button && !lastLeftStickButton) {
                if (currentRpmMode == RpmMode.AUTO_TARGETING) {
                    currentRpmMode = RpmMode.MANUAL_FIXED;
                } else {
                    currentRpmMode = RpmMode.AUTO_TARGETING;
                }
            }
            lastLeftStickButton = gamepad1.left_stick_button;

            // Update Robot Pose for calculations
            calculator.getRobotPose();

            // SWITCH CASE to determine target RPM
            switch (currentRpmMode) {
                case AUTO_TARGETING:
                    // 1. Calculate based on field position
                    calculatedRpm = calculator.calcVel();
                    // 2. Add the specific "Auto Offset" (Trim)
                    finalTargetRpm = calculatedRpm + rpmOffset;
                    break;

                case MANUAL_FIXED:
                    // 1. Use the static variable directly
                    finalTargetRpm = manualBaseRpm;
                    break;
            }

            // =========================================================
            // INPUT HANDLING: TRIM & ADJUSTMENT
            // =========================================================

            // D-Pad Up: Increase Speed
            if (gamepad1.dpad_up && !lastDpadUp) {
                if (currentRpmMode == RpmMode.AUTO_TARGETING) {
                    rpmOffset += 50; // In Auto, we adjust the offset
                } else {
                    manualBaseRpm += 50; // In Manual, we change the base
                }
            }
            lastDpadUp = gamepad1.dpad_up;

            // D-Pad Down: Decrease Speed
            if (gamepad1.dpad_down && !lastDpadDown) {
                if (currentRpmMode == RpmMode.AUTO_TARGETING) {
                    rpmOffset -= 50;
                } else {
                    manualBaseRpm -= 50;
                }
            }
            lastDpadDown = gamepad1.dpad_down;


            // =========================================================
            // DRIVE & AIM
            // =========================================================

            // Right Bumper: Toggle Aim Lock
            if (gamepad1.right_bumper && !lastRightBumper) {
                if (drive.isFaceTargetMode()) {
                    drive.disableFaceTarget();
                } else {
                    if (isRedAlliance) drive.enableFaceTarget(RED_GOAL_X, RED_GOAL_Y);
                    else drive.enableFaceTarget(BLUE_GOAL_X, BLUE_GOAL_Y);
                }
            }
            lastRightBumper = gamepad1.right_bumper;

            // Start: Auto-Park
            if (gamepad1.start && !lastStartButton) drive.driveToParking();
            lastStartButton = gamepad1.start;

            // Back: Reset Pose
            if (gamepad1.back) drive.resetPose(0,0,0);

            // Drive Controls
            boolean cancelAuto = Math.abs(gamepad1.left_stick_x) > 0.1 || Math.abs(gamepad1.left_stick_y) > 0.1;


            //fix
            drive.driveFieldCentric(-gamepad1.left_stick_x, -gamepad1.left_stick_y, -gamepad1.right_stick_x);
            drive.update(cancelAuto);


            // =========================================================
            // LAUNCHER ACTIONS
            // =========================================================

            // Button A: Pre-Spin using the Switch Case Result
            if (gamepad1.a) {
                launcher.preSpin(finalTargetRpm);
            }

            // Right Trigger: Shoot 1
            if (gamepad1.right_trigger > 0.5) {
                launcher.launch(finalTargetRpm, false);
            }
            // Left Trigger: Shoot 2
            else if (gamepad1.left_trigger > 0.5) {
                launcher.launch(finalTargetRpm, true);
            }

            // Button B: Stop
            if (gamepad1.b) launcher.stopAll();

            // Left Bumper: Advance Feeder
            if (gamepad1.left_bumper) launcher.advanceSecondArtifact();

            // X/Y: Unjam
            if (gamepad1.x) launcher.manualWheelForward();
            else if (gamepad1.y) launcher.manualWheelBack();
            else if (!launcher.getStateName().contains("FIRING") && !launcher.getStateName().contains("ADVANCING")) {
                launcher.manualWheelOff();
            }

            launcher.update();

            // =========================================================
            // TELEMETRY
            // =========================================================
            telemetry.addData("RPM MODE", currentRpmMode);

            if (currentRpmMode == RpmMode.AUTO_TARGETING) {
                telemetry.addData("Calc Base", "%.0f", calculatedRpm);
                telemetry.addData("Trim (+/-)", "%.0f", rpmOffset);
            } else {
                telemetry.addData("Fixed Setpoint", "%.0f", manualBaseRpm);
            }

            telemetry.addData("FINAL TARGET", "%.0f", finalTargetRpm);
            telemetry.addData("Real Vel L/R", "%.0f / %.0f", launcher.getLeftVelocity(), launcher.getRightVelocity());
            telemetry.update();
        }
    }
}