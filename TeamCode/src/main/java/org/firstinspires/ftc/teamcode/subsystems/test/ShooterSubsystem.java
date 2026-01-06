package org.firstinspires.ftc.teamcode.subsystems.test;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class ShooterSubsystem {
    private DcMotorEx shooterMotor;

    // == TUNING ==
    // STEP 1: Find Max RPM of your motor (e.g., 6000 RPM).
    // STEP 2: Calculate Max Ticks/Sec = (MaxRPM / 60) * TicksPerRev
    // STEP 3: Set kF = 32767 / Max_Ticks_Sec
    // STEP 4: Tune kP until it holds speed well. Leave I and D at 0.
    public static double kP = 10.0;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kF = 12.0;

    // Acceptable error in Ticks/Sec
    private static final double VELOCITY_TOLERANCE = 50;

    public void init(HardwareMap hwMap) {
        // Replace "shooter" with your config name
        shooterMotor = hwMap.get(DcMotorEx.class, "shooter");

        // Use FLOAT so the flywheel spins down naturally, reducing wear
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Reset Encoders
        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Important: RUN_USING_ENCODER is required for built-in velocity PID
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Apply PIDF Coefficients
        PIDFCoefficients pidf = new PIDFCoefficients(kP, kI, kD, kF);
        shooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
    }

    /**
     * Set the target flywheel velocity in Ticks Per Second.
     */
    public void setTargetVelocity(double ticksPerSecond) {
        shooterMotor.setVelocity(ticksPerSecond);
    }

    /**
     * Check if flywheel is at target speed.
     */
    public boolean isReady(double targetVelocity) {
        // Avoid checking readiness if target is 0
        if (targetVelocity < 10) return false;

        double currentVelocity = shooterMotor.getVelocity();
        return Math.abs(targetVelocity - currentVelocity) < VELOCITY_TOLERANCE;
    }

    public double getVelocity() {
        return shooterMotor.getVelocity();
    }

    public void stop() {
        shooterMotor.setVelocity(0);
    }
}