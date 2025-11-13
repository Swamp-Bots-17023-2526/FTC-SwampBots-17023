package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.PedroDrive;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;

@TeleOp(name = "TeleopRed", group = "Main")
public class TeleopRed extends OpMode {

    private PedroDrive drive;
    private Launcher launcher;
    private Intake intake;
    private Lift lift;

    // Edge detection
    private boolean prevY = false;
    private boolean prevX = false;
    private boolean prevDpadUp = false;
    private boolean prevDpadLeft = false;
    private boolean prevStart = false;
    private boolean prevRightStickButton = false;

    // Example auto-aim target (change to your real target coordinates)
    private static final double FACE_X = 60;   // field X of shooting target
    private static final double FACE_Y = 67;   // field Y of shooting target

    // Pose reset location (change to something meaningful for you)
    private static final double RESET_X = 9;
    private static final double RESET_Y = 9;
    private static final double RESET_H = 0.0; // radians

    // Launcher shot velocities (tune these)
    private static final double LOW_VELOCITY  = 1800;
    private static final double HIGH_VELOCITY = 2000;

    @Override
    public void init() {
        drive    = new PedroDrive(hardwareMap);
        launcher = new Launcher(hardwareMap);
        intake   = new Intake(hardwareMap);
        lift     = new Lift(hardwareMap);
        drive.setStartingPose(-10,-62,0);

        telemetry.addLine("TeleopRed initialized");
    }

    @Override
    public void start() {
        drive.startTeleOp();
    }

    @Override
    public void loop() {

        // ----------------- DRIVE (field-centric) -----------------
        double lx = gamepad1.left_stick_x;   // left stick: strafe
        double ly = gamepad1.left_stick_y;   // left stick: forward/back
        double rx = gamepad1.right_stick_x;  // right stick: rotation

        drive.driveFieldCentric(lx, ly, rx);

        // ----------------- INTAKE (A/B) -----------------
        // A button - intake
        // B button - outtake
        if (gamepad1.a) {
            intake.intakeIn();
        } else if (gamepad1.b) {
            intake.intakeOut();
        } else {
            intake.stop();
        }

        // ----------------- LAUNCHER CONTROLS -----------------

        // right trigger - spool flywheels (pre-spin, high velocity)
        if (gamepad1.right_trigger > 0.5) {
            launcher.preSpin(LOW_VELOCITY);
        }

        // Determine if there are two artifacts (right bumper held)
        boolean twoArtifacts = gamepad1.right_bumper;

        // left trigger - fire (lower velocity)
        if (gamepad1.left_trigger > 0.5) {
            launcher.launch(LOW_VELOCITY, twoArtifacts);
        }

        // left button (left bumper) - fire (higher velocity)
        if (gamepad1.left_bumper) {
            launcher.launch(HIGH_VELOCITY, twoArtifacts);
        }

        // Y button: manually hold wheel ON
        if (gamepad1.y) {
            launcher.manualWheelOn();
        } else {
            launcher.manualWheelOff();
        }


        // x button - stop launcher/intake and reset hammer position open
        if (gamepad1.x && !prevX) {
            launcher.stopAll();
            intake.stop();
        }

        // ----------------- LIFT / PARKING (DPAD) -----------------

        // dpad up - move to park (Pedro auto path)
        if (gamepad1.dpad_up && !prevDpadUp) {
            drive.driveToParking();
        }

        // dpad right - lift the robot (extend lift)
        if (gamepad1.dpad_right) {
            lift.extendToPark();
        }

        // dpad down - stop auto moving (cancel Pedro auto path)
        boolean cancelAuto = gamepad1.dpad_down;

        // dpad left - kill lift
        if (gamepad1.dpad_left && !prevDpadLeft) {
            lift.stopAll();
        }

        // ----------------- AUTO AIM (right stick pressed) -----------------

        // right joystick pressed - toggle auto aim (face target)
        if (gamepad1.right_stick_button && !prevRightStickButton) {
            if (drive.isFaceTargetMode()) {
                drive.disableFaceTarget();
            } else {
                drive.enableFaceTarget(FACE_X, FACE_Y);
            }
        }

        // ----------------- POSE RESET (start button) -----------------

        // start button - reset position
        if (gamepad1.start && !prevStart) {
            drive.resetPose(RESET_X, RESET_Y, RESET_H);
        }

        // ----------------- UPDATE SUBSYSTEMS -----------------
        drive.update(cancelAuto);
        launcher.update();
        intake.update();
        lift.update();

        // ----------------- TELEMETRY -----------------
        telemetry.addLine("=== DRIVE ===");
        telemetry.addData("Field Centric", true);
        telemetry.addData("Auto Path Active", drive.isAutomated());
        telemetry.addData("Face Target Mode", drive.isFaceTargetMode());
        telemetry.addData("Pose", "x=%.1f, y=%.1f, h=%.2f",
                drive.getPose().getX(),
                drive.getPose().getY(),
                drive.getPose().getHeading());

        telemetry.addLine();
        telemetry.addLine("=== CONTROLS (GAMEPAD1) ===");
        telemetry.addLine("Movement:  LS = move (field centric), RS X = rotate");
        telemetry.addLine("Intake:    A = intake, B = outtake");
        telemetry.addLine("Launcher:  RT = spool flywheels");
        telemetry.addLine("           LT = fire (LOW vel), LB = fire (HIGH vel), RB = hold if 2 artifacts");
        telemetry.addLine("           Y = advance 2 -> 1, X = stop launcher + intake, hammer open");
        telemetry.addLine("Lift:      Dpad Right = lift robot, Dpad Left = kill lift");
        telemetry.addLine("Parking:   Dpad Up = auto move to park, Dpad Down = stop auto moving");
        telemetry.addLine("Auto Aim:  Right stick press = toggle auto aim");
        telemetry.addLine("Pose:      Start = reset pose to preset location");

        telemetry.addLine();
        telemetry.addLine("=== SUBSYSTEM STATES ===");
        telemetry.addData("Launcher State", launcher.getStateName());
        telemetry.addData("Lift State", lift.getStateName());
        telemetry.addData("Intake State", intake.getState());

        telemetry.update();

        // ----------------- EDGE-STATE UPDATES -----------------
        prevY = gamepad1.y;
        prevX = gamepad1.x;
        prevDpadUp = gamepad1.dpad_up;
        prevDpadLeft = gamepad1.dpad_left;
        prevStart = gamepad1.start;
        prevRightStickButton = gamepad1.right_stick_button;
    }
}
