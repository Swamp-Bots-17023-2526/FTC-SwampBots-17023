package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * A subsystem class to manage a field-centric mecanum drive.
 * This class handles the initialization and control of the drive motors and IMU.
 */
public class FieldCentricDrive {

    // Drive motor objects
    private DcMotor leftFront;
    private DcMotor leftRear;
    private DcMotor rightFront;
    private DcMotor rightRear;

    // IMU for getting robot heading
    private IMU imu;

    /**
     * Constructor for the FieldCentricDrive subsystem.
     * @param hardwareMap The hardware map from the OpMode.
     * @param imu The pre-initialized IMU from the OpMode.
     */
    public FieldCentricDrive(HardwareMap hardwareMap, IMU imu) {
        // Initialize drive motors from the hardware map
        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftRear = hardwareMap.dcMotor.get("leftRear");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        rightRear = hardwareMap.dcMotor.get("rightRear");

        // The IMU is passed in from the main OpMode after it's been initialized
        this.imu = imu;

        // Set motor directions. This may need to be adjusted for your robot's specific build.
        // On many mecanum robots, the motors on one side need to be reversed.
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.FORWARD);


        // Set motors to brake when power is zero to prevent coasting
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Drives the robot using field-centric controls.
     * @param forward The desired forward/backward movement (-1 to 1).
     * @param strafe The desired left/right strafe movement (-1 to 1).
     * @param turn The desired rotational movement (-1 to 1).
     */
    public void drive(double forward, double strafe, double turn) {
        // Read the robot's current heading from the IMU.
        // The angle is negated because the FTC SDK's yaw convention is counter-clockwise positive.
        double heading = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the joystick inputs (forward and strafe) by the robot's heading.
        // This transforms the driver's perspective from robot-centric to field-centric.
        double rotatedForward = strafe * Math.sin(heading) + forward * Math.cos(heading);
        double rotatedStrafe = strafe * Math.cos(heading) - forward * Math.sin(heading);

        // Calculate the power for each wheel.
        // The "turn" input is added directly to create rotation.
        double leftFrontPower = rotatedForward + rotatedStrafe + turn;
        double leftRearPower = rotatedForward - rotatedStrafe + turn;
        double rightFrontPower = rotatedForward - rotatedStrafe - turn;
        double rightRearPower = rotatedForward + rotatedStrafe - turn;

        // Normalize the motor powers to ensure no value exceeds 1.0,
        // while maintaining the drive characteristics.
        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(turn), 1.0);
        leftFront.setPower(leftFrontPower / denominator);
        leftRear.setPower(leftRearPower / denominator);
        rightFront.setPower(rightFrontPower / denominator);
        rightRear.setPower(rightRearPower / denominator);
    }

    /**
     * Resets the IMU's heading to the current robot orientation.
     * This is useful for setting the "forward" direction at the start of a match.
     */
    public void resetIMU() {
        imu.resetYaw();
    }
}
