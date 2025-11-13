package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.Subsystems.FieldCentricDrive;

public class TestOp extends OpMode {


    DcMotor IntakeMotor;
    DcMotor RightFly;
    DcMotor LeftFly;

    //wheels
    DcMotor rightFront;
    DcMotor rightRear;
    DcMotor leftFront;
    DcMotor leftRear;

    IMU imu;

    // 1. Define the Hub's orientation with your specific directions.
    RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
    RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

    // 2. Create the orientation object from the directions.
    RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

    //for FC drive
    FieldCentricDrive DriveTrain;
    //variables for the forward strafe turn
    double forward;
    double strafe;
    double turn;


    //for toggle
    boolean toggle;

    @Override
    public void init(){
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        IntakeMotor = hardwareMap.dcMotor.get("IntakeMotor");
        RightFly = hardwareMap.dcMotor.get("RightFly");
        LeftFly = hardwareMap.dcMotor.get("LeftFly");

        //drive motors
        rightRear = hardwareMap.dcMotor.get("rightRear");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        leftRear = hardwareMap.dcMotor.get("leftRear");
        leftFront = hardwareMap.dcMotor.get("leftFront");

        //imu
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();

        //encoders (odom wheels)
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        //for the field centric
        DriveTrain = new FieldCentricDrive(hardwareMap, imu);
        forward = -gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        turn = gamepad1.right_stick_x;

    }

    //After button

    @Override
    public void loop(){
        DriveTrain.drive(forward, strafe, turn);

        //reset heading (x)
        if(gamepad1.x){
            DriveTrain.resetIMU();
        }
        //Intake
        if(gamepad1.a){

        }

    }
}
