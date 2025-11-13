package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Launcher;

@TeleOp(name = "LauncherTestTeleOp", group = "Test")
public class LauncherTestTeleOp extends OpMode {

    private Launcher launcher;

    // For edge detection so we don't spam commands every loop
    private boolean prevRightBumper = false;
    private boolean prevLeftBumper = false;
    private boolean prevY = false;
    private boolean prevLTPressed = false;

    // You can tune this to match your real shot speed
    private static final double TEST_VELOCITY = 1800;

    @Override
    public void init() {
        launcher = new Launcher(hardwareMap);
        telemetry.addLine("LauncherTestTeleOp initialized");
    }

    @Override
    public void loop() {

        // --------- Controls ---------

        boolean ltPressed = gamepad1.left_trigger > 0.5;

        // Pre-spin (hold/press left trigger)
        if (ltPressed && !prevLTPressed) {
            launcher.preSpin(TEST_VELOCITY);
        }

        // Right bumper -> launch assuming ONE artifact in system
        if (gamepad1.right_bumper && !prevRightBumper) {
            launcher.launch(TEST_VELOCITY, false);
        }

        // Left bumper -> launch assuming TWO artifacts in system
        if (gamepad1.left_bumper && !prevLeftBumper) {
            launcher.launch(TEST_VELOCITY, true);
        }

        // Y -> advance second artifact into launching position
        if (gamepad1.y && !prevY) {
            launcher.advanceSecondArtifact();
        }

        // X -> emergency stop all
        if (gamepad1.x) {
            launcher.stopAll();
        }

        // --------- Subsystem update ---------
        launcher.update();

        // --------- Telemetry ---------
        telemetry.addData("Launcher State", launcher.getStateName());
        telemetry.addData("Target Vel", "%.1f", launcher.getTargetVelocity());
        telemetry.addData("Left Vel", "%.1f", launcher.getLeftVelocity());
        telemetry.addData("Right Vel", "%.1f", launcher.getRightVelocity());
        telemetry.update();

        // Update edge-detection history
        prevRightBumper = gamepad1.right_bumper;
        prevLeftBumper  = gamepad1.left_bumper;
        prevY           = gamepad1.y;
        prevLTPressed   = ltPressed;
    }
}
