package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {

    private final DcMotor intakeMotor;

    // Power constants – tweak to suit your mechanism
    private static final double INTAKE_IN_POWER  = -1.0;
    private static final double INTAKE_OUT_POWER = 1.0;
    private static final double STOP_POWER       = 0.0;

    public enum State {
        STOPPED,
        INTAKING,
        OUTTAKING
    }

    private State state = State.STOPPED;

    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setPower(0);
    }

    // Turn the intake ON (pull artifacts inward)
    public void intakeIn() {
        intakeMotor.setPower(INTAKE_IN_POWER);
        state = State.INTAKING;
    }

    // Run intake backward to spit artifacts out
    public void intakeOut() {
        intakeMotor.setPower(INTAKE_OUT_POWER);
        state = State.OUTTAKING;
    }

    // Stop intake
    public void stop() {
        intakeMotor.setPower(STOP_POWER);
        state = State.STOPPED;
    }

    // Not strictly necessary, but included for consistency
    public void update() {
        // If your intake never needs timed states, this remains empty.
        // You can add sensors or jam detection here later.
    }

    public State getState() {
        return state;
    }
}
