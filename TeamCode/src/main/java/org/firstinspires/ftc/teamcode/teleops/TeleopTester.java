package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.PedroDrive;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;

@TeleOp(name = "TeleopTester", group = "Main")
public class TeleopTester extends OpMode {

    private PedroDrive drive;
    private Launcher launcher;
    private Intake intake;
    private Lift lift;

    // Edge detection
    private boolean prevA = false;
    private boolean prevY = false;
    private boolean prevB = false;

    // Example “face this point” and “parking” coordinates
    // (You will change these to your real field positions.)
    private static final double FACE_X = 72;   // e.g. center of a backdrop/target
    private static final double FACE_Y = 72;

    private static final double PARK_X = 60;   // parking cube position
    private static final double PARK_Y = 60;
    private static final double PARK_H = 0.0;  // heading at park (radians)

    @Override
    public void init() {
        drive    = new PedroDrive(hardwareMap);
        launcher = new Launcher(hardwareMap);
        intake   = new Intake(hardwareMap);
        lift     = new Lift(hardwareMap);

        // Configure parking pose if you want to override defaults here:
        drive.setParkingPose(PARK_X, PARK_Y, PARK_H);

        telemetry.addLine("MainTeleOpWithPedro initialized");
    }

    @Override
    public void start() {
        drive.startTeleOp();
    }

    @Override
    public void loop() {

        // ---- FIELD-CENTRIC DRIVE (left stick = translation, right stick = rotation) ----
        double lx = gamepad1.left_stick_x;
        double ly = gamepad1.left_stick_y;
        double rx = gamepad1.right_stick_x;

        drive.driveFieldCentric(lx, ly, rx);

        // ---- LAUNCHER CONTROLS (your existing scheme) ----
        if (gamepad1.left_trigger > 0.5) {
            launcher.preSpin(1800);
        }
        if (gamepad1.right_bumper) {
            launcher.launch(1800, false); // one artifact
        }
        if (gamepad1.left_bumper) {
            launcher.launch(1800, true);  // two artifacts
        }
        if (gamepad1.y && !prevY) {
            launcher.advanceSecondArtifact();
        }

        // ---- INTAKE CONTROLS ----
        if (gamepad1.a) {
            intake.intakeIn();
        } else if (gamepad1.b) {
            intake.intakeOut();
        } else {
            intake.stop();
        }

        // ---- LIFT CONTROLS (endgame example) ----
        if (gamepad1.dpad_up) {
            lift.extendToPark();
        }
        if (gamepad1.dpad_down) {
            lift.startManualRetract();
        } else if (!gamepad1.dpad_down && lift.getStateName().equals("MANUAL_RETRACT")) {
            lift.stopManualRetract();
        }

        // ---- PEDRO AUTOMATION BUTTONS ----

        // A: face a specific point wherever we are
        if (gamepad1.a && !prevA) {
            drive.facePoint(FACE_X, FACE_Y);
        }

        // Y: drive automatically to parking cube
        if (gamepad1.y && !prevY) {
            drive.driveToParking();
        }

        // B: cancel automated path/turn
        boolean cancelAuto = false;
        if (gamepad1.b && !prevB) {
            cancelAuto = true;
        }

        // ---- UPDATE SUBSYSTEMS ----
        drive.update(cancelAuto);
        launcher.update();
        intake.update();
        lift.update();

        // ---- TELEMETRY ----
        telemetry.addData("Drive Automated", drive.isAutomated());
        telemetry.addData("Drive Pose", drive.getPose());
        telemetry.addData("Lift State", lift.getStateName());
        telemetry.addData("Launcher State", launcher.getStateName());
        telemetry.update();

        // edge tracking
        prevA = gamepad1.a;
        prevY = gamepad1.y;
        prevB = gamepad1.b;
    }
}
