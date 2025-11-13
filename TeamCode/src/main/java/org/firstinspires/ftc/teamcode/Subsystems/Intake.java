package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.ams.AMSColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
public class Intake {

    DcMotor intakeMotor;

    //pushes the ball into the shooting place
    Servo feedSweep;

    private ElapsedTime timer = new ElapsedTime();


    public Intake(HardwareMap hwMap) {
        intakeMotor = hwMap.get(DcMotor.class, "intake");
        feedSweep = hwMap.get(Servo.class, "feedSweep");
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    public void push_into_shoot_pos(){
        feedSweep.setPosition(0.9);
    }

    public void spin() {
        intakeMotor.setPower(1);
    }

    public void stop() {
        intakeMotor.setPower(0);
    }


}
