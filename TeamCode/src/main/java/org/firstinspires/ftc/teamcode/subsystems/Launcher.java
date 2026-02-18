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
    // Velocity & tolerance
    // INCREASE THIS: 25 is often too tight. 100 allows for small fluctuations.

    // Tolerance
    private static final double VELOCITY_TOLERANCE = 100;

    // --- Timing ---
    // How long to keep spinning/firing during a shot
    private static final long FIRE_TIME_MS = 6000;
    // How long to spin the wheel to advance the second artifact
    private static final long FEED_TIME_MS = 3000;

    private final ElapsedTime timer = new ElapsedTime();

    // --- State Machine ---
    private enum State {
        IDLE,
        SPINNING_UP,
        FIRING_SINGLE,      // One artifact: Wheel only
        FIRING_TWO_FRONT,   // Two artifacts: Wheel only
        ADVANCING_SECOND    // Moving second artifact to position
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
        state = State.SPINNING_UP;
    }

    /**
     * Spins flywheels and triggers the firing sequence once speed is reached.
     */
    public void launch(double velocity, boolean twoArtifacts) {
        if (state != State.IDLE && state != State.SPINNING_UP) return;

        currentTwoArtifacts = twoArtifacts;
        launchRequested = true;

        if (state == State.IDLE) {
            targetVelocity = velocity;
            flywheelLeft.setVelocity(targetVelocity);
            flywheelRight.setVelocity(targetVelocity);
            state = State.SPINNING_UP;
        } else {
            // Already spinning, update target
            targetVelocity = velocity;
        }
    }

    /**
     * Advances the second artifact by spinning the wheel for FEED_TIME_MS.
     */
    public void advanceSecondArtifact() {
        if (state != State.IDLE) return;

        // Start spinning the wheel immediately
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
                // nothing to do
                break;

            case SPINNING_UP: {
                double leftVel  = flywheelLeft.getVelocity();
                double rightVel = flywheelRight.getVelocity();

                // Check if we are close enough to target
                boolean leftOk  = Math.abs(leftVel  - targetVelocity) <= VELOCITY_TOLERANCE;
                boolean rightOk = Math.abs(rightVel - targetVelocity) <= VELOCITY_TOLERANCE;

                // --- DEBUGGING HELP ---
                // If the wheel isn't spinning, it's stuck waiting here.
                // You can verify this by looking at telemetry in your OpMode.

                // CONDITION: Fire if speeds are good OR if we've been trying for > 3 seconds (safety fallback)
                // (Using a safety fallback prevents the auto from freezing if the battery is low)
                boolean timeOut = (timer.seconds() > 3.0 && launchRequested);

                if ((leftOk && rightOk) || timeOut) {

                    if (!launchRequested) {
                        // We are just pre-spinning. Keep flywheels on, stay in this state.
                        break;
                    }

                    // --- START THE SHOT ---
                    timer.reset(); // Reset timer to track the firing duration

                    if (currentTwoArtifacts) {
                        // Two Artifacts: Wheel ON
                        wheel.setPower(WHEEL_FEED_POWER);
                        state = State.FIRING_TWO_FRONT;
                    } else {
                        // Single Artifact: Wheel ON
                        wheel.setPower(WHEEL_FEED_POWER);
                        state = State.FIRING_SINGLE;
                    }
                }
                break;
            }

            case FIRING_SINGLE: {
                // Wait for the shot to complete
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

            case FIRING_TWO_FRONT: {
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