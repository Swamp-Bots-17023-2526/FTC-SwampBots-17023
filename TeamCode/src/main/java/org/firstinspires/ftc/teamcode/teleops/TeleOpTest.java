package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.geometry.Pose; // Import needed for getting current pose

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
    private Intake intake;

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
    private boolean lastButtonX = false;
    private boolean lastBackButton = false; // Added to prevent rapid resetting

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
        intake = new Intake(hardwareMap);

        // 2. Alliance Selection
        while (!isStarted() && !isStopRequested()) {

            // 2. Alliance Selection
            while (!isStarted() && !isStopRequested()) {
                if (gamepad1.a) isRedAlliance = true;
                if (gamepad1.x) isRedAlliance = false;

                telemetry.addLine("=== FULL ROBOT INIT ===");
                telemetry.addData("Alliance", isRedAlliance ? "RED" : "BLUE");
                telemetry.update();
            }

            // 3. Calculator Setup & STARTING POSE SETUP <--- ADD THIS
            if (isRedAlliance) {
                calculator.setRed();
                // Set Red Side Start (Example: Against the wall on Red side)
                // You must change these numbers to match where you actually place the robot!
                drive.setStartingPose(84.072, 83.448, Math.toRadians(45));
            } else {
                calculator.setBlue();
                // Set Blue Side Start
                drive.setStartingPose(9, 120, Math.toRadians(0));
            }

            calculator.setGoal();
            drive.startTeleOp();
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

            // Auto-Park (Start)
            if (gamepad1.start && !lastStartButton) drive.driveToParking();
            lastStartButton = gamepad1.start;

            // --- RESET HEADING LOGIC (Back Button) ---
            if (gamepad1.back && !lastBackButton) {
                // Get the current known position (X, Y, Heading)
                Pose currentPose = drive.getPose();

                // Reset to: Current X, Current Y, but forcing Heading to 0 (Forward)
                // This keeps your position on the field map but fixes gyro drift.
                drive.resetPose(currentPose.getX(), currentPose.getY(), 0);

                // If you prefer to reset to 180 (facing back) for Red Alliance, use:
                // double resetAngle = isRedAlliance ? Math.toRadians(180) : Math.toRadians(0);
                // drive.resetPose(currentPose.getX(), currentPose.getY(), resetAngle);
            }
            lastBackButton = gamepad1.back;

            // Drive Command
            boolean cancelAuto = Math.abs(gamepad1.left_stick_x) > 0.1 || Math.abs(gamepad1.left_stick_y) > 0.1;
            // Note: Added negative signs to stick inputs if your drive feels inverted, remove if not needed.
            drive.driveFieldCentric(-gamepad1.left_stick_x, -gamepad1.left_stick_y, -gamepad1.right_stick_x);
            drive.update(cancelAuto);

            // --- 3. INTAKE CONTROLS ---

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
            if (gamepad1.y) {
                intake.intakeOut();
            } else if (!gamepad1.x && intake.getState() == Intake.State.OUTTAKING) {
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
                intake.stop();
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

            // D-Pad LEFT/RIGHT: Manual Launcher Unjam
            if (gamepad1.dpad_left) {
                launcher.manualWheelForward();
            } else if (gamepad1.dpad_right) {
                launcher.manualWheelBack();
            } else {
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