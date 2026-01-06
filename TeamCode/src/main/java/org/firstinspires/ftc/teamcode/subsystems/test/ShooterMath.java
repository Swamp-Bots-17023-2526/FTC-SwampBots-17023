package org.firstinspires.ftc.teamcode.subsystems.test;

import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class ShooterMath {

    // == VELOCITY TUNING ==
    // You must test this on the field!
    // Place robot at 24 inches -> Find RPM that scores.
    // Place robot at 72 inches -> Find RPM that scores.
    private static final double DIST_NEAR = 24.0; // inches
    private static final double RPM_NEAR = 1500.0; // ticks per sec

    private static final double DIST_FAR = 72.0;  // inches
    private static final double RPM_FAR = 2600.0; // ticks per sec

    /**
     * Calculates the Field (X, Y) position of the AprilTag.
     * PedroDrive needs this global coordinate to aim correctly.
     */
    public static double[] calcTagCoordinates(Pose robotPose, AprilTagDetection detection) {
        // 1. Get Range in Inches (Your vision subsystem uses CM)
        double rangeInches = detection.ftcPose.range / 2.54;

        // 2. Get Bearing (Angle of tag relative to camera center)
        double bearingRad = Math.toRadians(detection.ftcPose.bearing);

        // 3. Calculate Global Angle
        // Robot Heading + Bearing = Direction of Tag in World
        double tagWorldAngle = robotPose.getHeading() + bearingRad;

        // 4. Calculate Tag X and Y
        double tagX = robotPose.getX() + (rangeInches * Math.cos(tagWorldAngle));
        double tagY = robotPose.getY() + (rangeInches * Math.sin(tagWorldAngle));

        return new double[]{tagX, tagY};
    }

    /**
     * Linear Interpolation: Calculates needed motor speed based on distance.
     */
    public static double calcShooterVelocity(AprilTagDetection detection) {
        if (detection == null) return 0.0;

        double dist = detection.ftcPose.range / 2.54; // Convert CM to Inches

        // Clamp distance (don't calculate for values outside our test range)
        if (dist <= DIST_NEAR) return RPM_NEAR;
        if (dist >= DIST_FAR)  return RPM_FAR;

        // Linear equation: y = mx + b
        double slope = (RPM_FAR - RPM_NEAR) / (DIST_FAR - DIST_NEAR);
        return RPM_NEAR + (slope * (dist - DIST_NEAR));
    }
}