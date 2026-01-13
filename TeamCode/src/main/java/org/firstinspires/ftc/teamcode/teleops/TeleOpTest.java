package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.PedroDrive;
import org.firstinspires.ftc.teamcode.subsystems.test.PedroCalculate;

@TeleOp(name = "Full Robot TeleOp", group = "Competition")
public class TeleOpTest extends LinearOpMode {

    // --- Subsystems ---
    private PedroDrive drive;
    private Launcher launcher;
    private PedroCalculate calculator;
    private Intake intake; // <--- NEW

    // --- Enums ---
    private enum RpmMode { AUTO_TARGETING, MANUAL_FIXED }
    private RpmMode currentRpmMode = RpmMode.AUTO_TARGETING;

    // --- State Variables ---
    private boolean isRedAlliance = true;
    private double calculatedRpm = 0.0;
    private double manualBaseRpm = 1700.0;
    private double rpmOffset = 0.0;
    private double finalTargetRpm = 0.0;

    // --- Toggle Memory ---
    private boolean lastRightBumper = false;
    private boolean lastStartButton = false;
    private boolean lastLeftStickButton = false;
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastButtonX = false; // For Intake Toggle

    // --- Goal Coordinates ---
    private static final double RED_GOAL_X = 138;
    private static final double RED_GOAL_Y = 138;
    private static final double BLUE_GOAL_X = 4.5;
    private static final double BLUE_GOAL_Y = 139;

    @Override
    public void runOpMode() throws InterruptedException {
        // 1. Initialize All Subsystems
        drive = new PedroDrive(hardwareMap);
        launcher = new Launcher(hardwareMap);
        calculator = new PedroCalculate(hardwareMap);
        intake = new Intake(hardwareMap); // <--- NEW

        // 2. Alliance Selection
        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.a) isRedAlliance = true;
            if (gamepad1.x) isRedAlliance = false;

            telemetry.addLine("=== FULL ROBOT INIT ===");
            telemetry.addData("Alliance", isRedAlliance ? "RED" : "BLUE");
            telemetry.addLine("Controls: X=Intake, Triggers=Shoot");
            telemetry.update();
        }

        // 3. Calculator Setup
        if (isRedAlliance) calculator.setRed();
        else calculator.setBlue();
        calculator.setGoal();

        drive.startTeleOp();

        // 4. Main Loop
        while (opModeIsActive()) {

            // --- 1. CALCULATIONS ---
            calculator.getRobotPose();

            // Toggle RPM Mode (L3)
            if (gamepad1.left_stick_button && !lastLeftStickButton) {
                currentRpmMode = (currentRpmMode == RpmMode.AUTO_TARGETING) ?
                        RpmMode.MANUAL_FIXED : RpmMode.AUTO_TARGETING;
            }
            lastLeftStickButton = gamepad1.left_stick_button;

            // Calculate Target RPM
            switch (currentRpmMode) {
                case AUTO_TARGETING:
                    calculatedRpm = calculator.calcVel();
                    finalTargetRpm = calculatedRpm + rpmOffset;
                    break;
                case MANUAL_FIXED:
                    finalTargetRpm = manualBaseRpm;
                    break;
            }

            // --- 2. DRIVING & AIMING ---

            // Aim Lock (Right Bumper)
            if (gamepad1.right_bumper && !lastRightBumper) {
                if (drive.isFaceTargetMode()) drive.disableFaceTarget();
                else {
                    if (isRedAlliance) drive.enableFaceTarget(RED_GOAL_X, RED_GOAL_Y);
                    else drive.enableFaceTarget(BLUE_GOAL_X, BLUE_GOAL_Y);
                }
            }
            lastRightBumper = gamepad1.right_bumper;

            // Auto-Park (Start) & Reset Pose (Back)
            if (gamepad1.start && !lastStartButton) drive.driveToParking();
            lastStartButton = gamepad1.start;
            if (gamepad1.back) drive.resetPose(0,0,0);

            // Drive Command
            boolean cancelAuto = Math.abs(gamepad1.left_stick_x) > 0.1 || Math.abs(gamepad1.left_stick_y) > 0.1;
            drive.driveFieldCentric(-gamepad1.left_stick_x, -gamepad1.left_stick_y, -gamepad1.right_stick_x);
            drive.update(cancelAuto);

            // --- 3. INTAKE CONTROLS (NEW) ---

            // Button X: Toggle Intake IN / OFF
            if (gamepad1.x && !lastButtonX) {
                if (intake.getState() == Intake.State.INTAKING) {
                    intake.stop();
                } else {
                    intake.intakeIn();
                }
            }
            lastButtonX = gamepad1.x;

            // Button Y: Hold to OUTTAKE (Reverse)
            // This overrides the toggle. When released, it stops.
            if (gamepad1.y) {
                intake.intakeOut();
            } else if (!gamepad1.x && intake.getState() == Intake.State.OUTTAKING) {
                // If we released Y, and we aren't pressing X, stop.
                intake.stop();
            }

            // --- 4. LAUNCHER CONTROLS ---

            // Button A: Spin Up
            if (gamepad1.a) launcher.preSpin(finalTargetRpm);

            // Triggers: Fire
            if (gamepad1.right_trigger > 0.5) launcher.launch(finalTargetRpm, false);
            else if (gamepad1.left_trigger > 0.5) launcher.launch(finalTargetRpm, true);

            // Left Bumper: Advance Second Artifact
            if (gamepad1.left_bumper) launcher.advanceSecondArtifact();

            // Button B: STOP ALL (Safety)
            if (gamepad1.b) {
                launcher.stopAll();
                intake.stop(); // Stop intake too!
            }

            // --- 5. ADJUSTMENTS (D-PAD) ---

            // D-Pad UP/DOWN: RPM Adjustment
            if (gamepad1.dpad_up && !lastDpadUp) {
                if (currentRpmMode == RpmMode.AUTO_TARGETING) rpmOffset += 50;
                else manualBaseRpm += 50;
            }
            lastDpadUp = gamepad1.dpad_up;

            if (gamepad1.dpad_down && !lastDpadDown) {
                if (currentRpmMode == RpmMode.AUTO_TARGETING) rpmOffset -= 50;
                else manualBaseRpm -= 50;
            }
            lastDpadDown = gamepad1.dpad_down;

            // D-Pad LEFT/RIGHT: Manual Launcher Unjam (Moved from X/Y)
            if (gamepad1.dpad_left) {
                launcher.manualWheelForward();
            } else if (gamepad1.dpad_right) {
                launcher.manualWheelBack();
            } else {
                // Stop manual wheel if not firing/advancing
                String lState = launcher.getStateName();
                if (!lState.contains("FIRING") && !lState.contains("ADVANCING")) {
                    launcher.manualWheelOff();
                }
            }

            // Update Systems
            launcher.update();
            intake.update();

            // --- 6. TELEMETRY ---
            telemetry.addData("RPM Mode", currentRpmMode);
            telemetry.addData("Intake State", intake.getState());
            telemetry.addData("Final Target RPM", "%.0f", finalTargetRpm);
            telemetry.addData("Launcher L/R", "%.0f / %.0f", launcher.getLeftVelocity(), launcher.getRightVelocity());
            telemetry.update();
        }
    }
}