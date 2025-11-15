package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lift {

    private final DcMotorEx liftMotor;

    // Power levels – tune if needed
    private static final double UP_POWER       = 1.0;   // D-pad right
    private static final double DOWN_POWER     = -0.1;  // D-pad down (gentle retract)
    private static final double STOP_POWER     = 0.0;

    public enum State {
        STOPPED,
        MOVING_UP,
        MOVING_DOWN
    }

    private State state = State.STOPPED;

    public Lift(HardwareMap hardwareMap) {
        liftMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "lift");

        // Open-loop, no encoder-based control
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setPower(0.0);
    }

    // ===== COMMANDS =====

    /** Run lift upward at full power (used for D-pad right). */
    public void moveUp() {
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setPower(UP_POWER);
        state = State.MOVING_UP;
    }

    /** Run lift downward slowly (used for D-pad down). */
    public void moveDownSlow() {
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setPower(DOWN_POWER);
        state = State.MOVING_DOWN;
    }

    /** Stop lift and hold with BRAKE. */
    public void stopAll() {
        liftMotor.setPower(STOP_POWER);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        state = State.STOPPED;
    }

    // ===== UPDATE LOOP =====
    // Nothing to do each cycle; left for symmetry with other subsystems.
    public void update() {
        // no-op for now
    }

    // ===== TELEMETRY HELPERS =====

    public String getStateName() {
        return state.toString();
    }

    public int getCurrentPosition() {
        return liftMotor.getCurrentPosition();
    }
}
