package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Launcher {

    // --- Hardware ---
    private final CRServo wheel;
    private final DcMotorEx flywheelLeft, flywheelRight;

    // --- Tuning Constants ---

    // Wheel Power
    private static final double WHEEL_FEED_POWER = 1.0;

    // Flywheel PIDF
    private static final double FLYWHEEL_P = 12;
    private static final double FLYWHEEL_I = 3;
    private static final double FLYWHEEL_D = 3;
    private static final double FLYWHEEL_F = 1;

    // Tolerance
    private static final double VELOCITY_TOLERANCE = 75;

    // --- Timing ---
    private static final long FIRE_TIME_MS = 6000;
    private static final long FEED_TIME_MS = 3000;

    private final ElapsedTime timer = new ElapsedTime();

    // --- State Machine ---
    private enum State {
        IDLE,
        SPINNING_UP,
        FIRING_SINGLE,
        FIRING_TWO_FRONT,
        ADVANCING_SECOND
    }

    private State state = State.IDLE;
    private boolean currentTwoArtifacts = false;
    private boolean launchRequested = false;
    private double targetVelocity = 0;


    // --- Constructor ---
    public Launcher(HardwareMap hardwareMap) {
        wheel         = hardwareMap.get(CRServo.class, "wheel");
        flywheelLeft  = hardwareMap.get(DcMotorEx.class, "leftFly");
        flywheelRight = hardwareMap.get(DcMotorEx.class, "rightFly");

        initializeMotors();
    }

    private void initializeMotors() {
        wheel.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelRight.setDirection(DcMotorSimple.Direction.REVERSE);

        flywheelLeft.setVelocityPIDFCoefficients(FLYWHEEL_P, FLYWHEEL_I, FLYWHEEL_D, FLYWHEEL_F);
        flywheelRight.setVelocityPIDFCoefficients(FLYWHEEL_P, FLYWHEEL_I, FLYWHEEL_D, FLYWHEEL_F);
    }


    // --- Command Methods ---

    /**
     * Spins flywheels to target velocity but does NOT fire.
     */
    public void preSpin(double velocity) {
        if (state != State.IDLE && state != State.SPINNING_UP) return;

        targetVelocity = velocity;
        flywheelLeft.setVelocity(targetVelocity);
        flywheelRight.setVelocity(targetVelocity);

        launchRequested = false;
        timer.reset(); // FIX: Reset timer so it tracks time since spin-up started
        state = State.SPINNING_UP;
    }

    /**
     * Spins flywheels and triggers the firing sequence once speed is reached.
     */
    public void launch(double velocity, boolean twoArtifacts) {
        if (state != State.IDLE && state != State.SPINNING_UP) return;

        currentTwoArtifacts = twoArtifacts;
        launchRequested = true;
        targetVelocity = velocity;

        flywheelLeft.setVelocity(targetVelocity);
        flywheelRight.setVelocity(targetVelocity);

        timer.reset(); // FIX: Reset timer so the 3-second timeout is accurate
        state = State.SPINNING_UP;
    }

    /**
     * Advances the second artifact by spinning the wheel for FEED_TIME_MS.
     */
    public void advanceSecondArtifact() {
        if (state != State.IDLE) return;

        wheel.setPower(WHEEL_FEED_POWER);
        timer.reset();
        state = State.ADVANCING_SECOND;
    }

    public void stopAll() {
        flywheelLeft.setVelocity(0);
        flywheelRight.setVelocity(0);
        wheel.setPower(0);
        launchRequested = false;
        currentTwoArtifacts = false;
        state = State.IDLE;
    }

    // --- Manual Controls (Safety: Only work in IDLE) ---
    public void manualWheelForward() {
        if (state == State.IDLE) wheel.setPower(WHEEL_FEED_POWER);
    }

    public void manualWheelBack() {
        if (state == State.IDLE) wheel.setPower(-WHEEL_FEED_POWER);
    }

    public void manualWheelOff() {
        if (state == State.IDLE) wheel.setPower(0);
    }


    // --- Update Loop ---

    public void update() {
        switch (state) {
            case IDLE:
                break;

            case SPINNING_UP: {
                double leftVel  = flywheelLeft.getVelocity();
                double rightVel = flywheelRight.getVelocity();

                // Check if we are close enough to target
                boolean leftOk  = Math.abs(leftVel  - targetVelocity) <= VELOCITY_TOLERANCE;
                boolean rightOk = Math.abs(rightVel - targetVelocity) <= VELOCITY_TOLERANCE;

                // Safety fallback: Fire if we've been waiting for > 3 seconds since launch() was called
                boolean timeOut = (timer.seconds() > 3.0 && launchRequested);

                if ((leftOk && rightOk) || timeOut) {

                    if (!launchRequested) {
                        // We are just pre-spinning. Keep resetting the timer to prevent
                        // an instant timeout if launch() is called later.
                        timer.reset();
                        break;
                    }

                    // --- START THE SHOT ---
                    timer.reset();

                    if (currentTwoArtifacts) {
                        wheel.setPower(WHEEL_FEED_POWER);
                        state = State.FIRING_TWO_FRONT;
                    } else {
                        wheel.setPower(WHEEL_FEED_POWER);
                        state = State.FIRING_SINGLE;
                    }
                }
                break;
            }

            case FIRING_SINGLE:
            case FIRING_TWO_FRONT: { // Merged these two cases since the logic was identical
                if (timer.milliseconds() >= FIRE_TIME_MS) {
                    wheel.setPower(0);
                    flywheelLeft.setVelocity(0);
                    flywheelRight.setVelocity(0);

                    launchRequested = false;
                    currentTwoArtifacts = false;
                    state = State.IDLE;
                }
                break;
            }

            case ADVANCING_SECOND: {
                if (timer.milliseconds() >= FEED_TIME_MS) {
                    wheel.setPower(0);
                    state = State.IDLE;
                }
                break;
            }
        }
    }

    // --- Helpers ---

    public boolean isBusy() {
        return state != State.IDLE;
    }

    public String getStateName() {
        return state.toString();
    }

    public double getTargetVelocity() { return targetVelocity; }
    public double getLeftVelocity()   { return flywheelLeft.getVelocity(); }
    public double getRightVelocity()  { return flywheelRight.getVelocity(); }
}