package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode; // Extending OpMode, not LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import java.util.List;

@Autonomous(name = "AprilTagTest ")
public class AprilTagTest extends OpMode {

    VisionSubsystem aprilTagWebcam = new VisionSubsystem();

    @Override
    public void init(){
        aprilTagWebcam.init(hardwareMap, telemetry);
        telemetry.update();
    }
    @Override
    public void loop(){
        //upd visionPortal

        aprilTagWebcam.update();

        // 2. Try to find Tag 21
        AprilTagDetection id21 = aprilTagWebcam.getSpecificTag(21);

        // 3. Display formatted telemetry (Your subsystem handles null here safely)
        aprilTagWebcam.displayDetectedTelemetry(id21);

        // 4. FIX: Only call .toString() if the tag exists
        if (id21 != null) {
            telemetry.addData("id21 String", id21.toString());
        } else {
            telemetry.addData("id21 String", "Tag 21 not detected");
        }


       /* aprilTagWebcam.update();
        AprilTagDetection id21 = aprilTagWebcam.getSpecificTag(21);
        aprilTagWebcam.displayDetectedTelemetry(id21);
        telemetry.addData("id21 String", id21.toString());
    */
    }
}