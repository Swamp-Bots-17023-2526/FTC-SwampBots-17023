package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lift {

    private final DcMotorEx liftMotor;

    // === PRESET POSITIONS (TUNE THESE!) ===
    // Encoder tick value for your "car lift" / hang height.
    // You MUST measure & tune this on the robot.
    public static final int LIFT_HANG_POSITION = 3000;  // placeholder

    // Power levels
    private static final double AUTO_EXTEND_POWER    = 0.9;   // power while going UP to hang
    private static final double MANUAL_RETRACT_POWER = -0.5;  // power while pulling DOWN manually

    // Tolerance in ticks for considering "at position"
    private static final int POSITION_TOLERANCE = 20;

    private enum State {
        IDLE,
        EXTENDING_TO_HANG,
        MANUAL_RETRACT
    }

    private State state = State.IDLE;

    public Lift(HardwareMap hardwareMap) {
        liftMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "lift");

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setPower(0.0);
    }

    // ========= PUBLIC COMMANDS =========

    /** Command the lift to move to the hang height using RUN_TO_POSITION. */
    public void extendToPark() {
        // Only start if not already under manual override
        if (state == State.MANUAL_RETRACT) return;

        liftMotor.setTargetPosition(LIFT_HANG_POSITION);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(AUTO_EXTEND_POWER);

        state = State.EXTENDING_TO_HANG;
    }

    /**
     * Begin manual retraction override.
     * Typically called while a button is HELD.
     */
    public void startManualRetract() {
        // Switch to open-loop control
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor.setPower(MANUAL_RETRACT_POWER);
        state = State.MANUAL_RETRACT;
    }

    /** Stop manual retraction and hold in place (BRAKE). */
    public void stopManualRetract() {
        liftMotor.setPower(0.0);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        state = State.IDLE;

        // Optionally: re-sync encoder if you care about absolute position afterwards
        // liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /** Hard stop everything, used for safety/emergency. */
    public void stopAll() {
        liftMotor.setPower(0.0);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        state = State.IDLE;
    }

    // ========= UPDATE LOOP (CALL EVERY ITERATIVE CYCLE) =========

    public void update() {
        switch (state) {
            case EXTENDING_TO_HANG: {
                int currentPos = liftMotor.getCurrentPosition();
                int error = Math.abs(LIFT_HANG_POSITION - currentPos);

                // Either motor reports not busy or we're within an error band
                if (!liftMotor.isBusy() || error <= POSITION_TOLERANCE) {
                    liftMotor.setPower(0.0);
                    liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    state = State.IDLE;
                }
                break;
            }

            case MANUAL_RETRACT:
                // Power is set in startManualRetract(); nothing to do here.
                break;

            case IDLE:
            default:
                // Do nothing.
                break;
        }
    }

    // ========= TELEMETRY HELPERS =========

    public int getCurrentPosition() {
        return liftMotor.getCurrentPosition();
    }

    public String getStateName() {
        return state.toString();
    }
}
