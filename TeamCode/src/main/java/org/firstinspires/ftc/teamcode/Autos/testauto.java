package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "Simple 2 Second Drive", group = "Test")
public class testauto extends LinearOpMode {

    @Override
    public void runOpMode() {
        // 1. Initialize Motors using the specific names you provided
        // Note: The variable name is logical (leftFront), but it grabs the config name you asked for ("leftRear")
        DcMotor leftFront  = hardwareMap.get(DcMotor.class, "leftRear");
        DcMotor leftRear   = hardwareMap.get(DcMotor.class, "leftFront");
        DcMotor rightFront = hardwareMap.get(DcMotor.class, "rightRear");
        DcMotor rightRear  = hardwareMap.get(DcMotor.class, "rightFront");

        // 2. Set Directions (Reverse left side so robot moves forward)
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Ready to Run");
        telemetry.update();

        // 3. Wait for the start button
        waitForStart();

        if (opModeIsActive()) {
            // 4. Turn motors ON (50% power)
            leftFront.setPower(1);
            leftRear.setPower(1);
            rightFront.setPower(1);
            rightRear.setPower(1);

            // 5. Wait for 2 seconds (2000 milliseconds)
            sleep(500);

            // 6. Turn motors OFF
            leftFront.setPower(0);
            leftRear.setPower(0);
            rightFront.setPower(0);
            rightRear.setPower(0);
            leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        }
    }
}