package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Launcher {

    // Hardware
    private final Servo hammer, feedsweep;
    private final CRServo wheel;
    private final DcMotorEx flywheelLeft, flywheelRight;

    // Hammer positions – tune on robot
    private static final double HAMMER_OPEN   = 0.9;
    private static final double HAMMER_PRIMED = 0.7;
    private static final double HAMMER_FIRE   = 0.6;

    // Feedsweep positions – guessed; tune on robot
    private static final double FEEDSWEEP_RETRACTED = 0.0;
    private static final double FEEDSWEEP_FEED      = 0.3;

    // Wheel power for feeding
    private static final double WHEEL_FEED_POWER = 1.0;

    // Flywheel PIDF – from your original code
    private static final double FLYWHEEL_P = 12;
    private static final double FLYWHEEL_I = 3;
    private static final double FLYWHEEL_D = 3;
    private static final double FLYWHEEL_F = 1;

    // Velocity & tolerance
    private double targetVelocity = 0;
    private static final double VELOCITY_TOLERANCE = 25; // widen if too picky

    // Timing (ms) – tune by feel
    private static final long FIRE_TIME_MS = 200; // how long hammer/wheel act for a shot
    private static final long FEED_TIME_MS = 250; // how long to advance second artifact

    private final ElapsedTime timer = new ElapsedTime();

    // --- State machine ---

    private enum State {
        IDLE,
        SPINNING_UP,
        FIRING_SINGLE,      // one artifact: hammer + wheel
        FIRING_TWO_FRONT,   // two artifacts: wheel only, front artifact fired
        ADVANCING_SECOND    // move second artifact into launching position
    }

    private State state = State.IDLE;

    // Whether the current shot was requested as "two artifacts"
    private boolean currentTwoArtifacts = false;

    // Whether we’ve actually been asked to fire (vs just pre-spin)
    private boolean launchRequested = false;


    // --------- Constructor / init ---------

    public Launcher(HardwareMap hardwareMap) {
        hammer     = hardwareMap.get(Servo.class, "hammer");
        feedsweep  = hardwareMap.get(Servo.class, "feedsweep");
        wheel      = hardwareMap.get(CRServo.class, "wheel");
        flywheelLeft  = hardwareMap.get(DcMotorEx.class, "leftFly");
        flywheelRight = hardwareMap.get(DcMotorEx.class, "rightFly");

        initializeMotors();
        initializeServos();
    }

    private void initializeMotors() {
        wheel.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelRight.setDirection(DcMotorSimple.Direction.REVERSE);

        flywheelLeft.setVelocityPIDFCoefficients(
                FLYWHEEL_P, FLYWHEEL_I, FLYWHEEL_D, FLYWHEEL_F
        );
        flywheelRight.setVelocityPIDFCoefficients(
                FLYWHEEL_P, FLYWHEEL_I, FLYWHEEL_D, FLYWHEEL_F
        );
    }

    private void initializeServos() {
        hammer.setPosition(HAMMER_OPEN);
        feedsweep.setPosition(FEEDSWEEP_RETRACTED);
    }

    // --------- Public command methods ---------

    /**
     * Pre-spin the flywheels to a desired velocity without firing.
     * Non-blocking. You can later call launch(...) to actually shoot.
     */
    public void preSpin(double velocity) {
        // Allow pre-spin if idle or already spinning up
        if (state != State.IDLE && state != State.SPINNING_UP) return;

        targetVelocity = velocity;

        flywheelLeft.setVelocity(targetVelocity);
        flywheelRight.setVelocity(targetVelocity);

        // Prepare hammer so that launch can fire quickly
        hammer.setPosition(HAMMER_PRIMED);

        // We are spinning, but no shot is yet requested
        launchRequested = false;
        state = State.SPINNING_UP;
    }

    /**
     * Request a launch.
     *
     * @param velocity     flywheel target velocity (ticks/sec)
     * @param twoArtifacts true  -> two artifacts in system: wheel only fires front artifact
     *                     false -> single artifact: hammer + wheel fire together
     */
    public void launch(double velocity, boolean twoArtifacts) {
        // If we're in the middle of feeding or firing, ignore
        if (state != State.IDLE && state != State.SPINNING_UP) return;

        currentTwoArtifacts = twoArtifacts;
        launchRequested = true;

        if (state == State.IDLE) {
            // Start from rest
            targetVelocity = velocity;
            flywheelLeft.setVelocity(targetVelocity);
            flywheelRight.setVelocity(targetVelocity);
            hammer.setPosition(HAMMER_PRIMED);
            state = State.SPINNING_UP;
        } else if (state == State.SPINNING_UP) {
            // Already spinning: just adjust target if needed
            targetVelocity = velocity;
        }
    }

    /**
     * Move the second artifact into the launching position using
     * the wheel and feedsweep, with hammer opened to allow movement.
     */
    public void advanceSecondArtifact() {
        if (state != State.IDLE) return;

        hammer.setPosition(HAMMER_OPEN);
        wheel.setPower(WHEEL_FEED_POWER);
        feedsweep.setPosition(FEEDSWEEP_FEED);

        timer.reset();
        state = State.ADVANCING_SECOND;
    }

    /** Option to stop everything quickly from TeleOp. */
    public void stopAll() {
        flywheelLeft.setVelocity(0);
        flywheelRight.setVelocity(0);
        wheel.setPower(0);
        feedsweep.setPosition(FEEDSWEEP_RETRACTED);
        hammer.setPosition(HAMMER_OPEN);

        launchRequested = false;
        currentTwoArtifacts = false;
        state = State.IDLE;
    }

    // --------- Update loop (call every cycle) ---------

    public void update() {
        switch (state) {
            case IDLE:
                // nothing to do
                break;

            case SPINNING_UP: {
                double leftVel  = flywheelLeft.getVelocity();
                double rightVel = flywheelRight.getVelocity();

                boolean leftAtSpeed  = Math.abs(leftVel  - targetVelocity) <= VELOCITY_TOLERANCE;
                boolean rightAtSpeed = Math.abs(rightVel - targetVelocity) <= VELOCITY_TOLERANCE;

                if (leftAtSpeed && rightAtSpeed) {
                    // If we were only pre-spinning, just hold speed
                    if (!launchRequested) {
                        // stay in SPINNING_UP, nothing else
                        break;
                    }

                    // Otherwise we actually want to shoot now
                    timer.reset();

                    if (currentTwoArtifacts) {
                        // TWO artifacts: wheel only pushes the front artifact out
                        wheel.setPower(WHEEL_FEED_POWER);
                        state = State.FIRING_TWO_FRONT;
                    } else {
                        // ONE artifact: hammer and wheel both act
                        hammer.setPosition(HAMMER_FIRE);
                        wheel.setPower(WHEEL_FEED_POWER);
                        state = State.FIRING_SINGLE;
                    }
                }
                break;
            }

            case FIRING_SINGLE: {
                if (timer.milliseconds() >= FIRE_TIME_MS) {
                    // Stop wheel and reset hammer
                    wheel.setPower(0);
                    hammer.setPosition(HAMMER_OPEN);   // or HAMMER_PRIMED if you prefer

                    // Stop flywheels after shot
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
                    // Stop wheel; front artifact should be gone
                    wheel.setPower(0);

                    // Stop flywheels after shot
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
                    // Stop feed mechanisms
                    wheel.setPower(0);
                    feedsweep.setPosition(FEEDSWEEP_RETRACTED);

                    // Second artifact should now be in launch position
                    hammer.setPosition(HAMMER_PRIMED);

                    state = State.IDLE;
                }
                break;
            }
        }
    }

    // --------- Telemetry helpers ---------

    public String getStateName() {
        return state.toString();
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public double getLeftVelocity() {
        return flywheelLeft.getVelocity();
    }

    public double getRightVelocity() {
        return flywheelRight.getVelocity();
    }
}
