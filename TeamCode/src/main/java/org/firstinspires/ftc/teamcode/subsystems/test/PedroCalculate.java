package org.firstinspires.ftc.teamcode.subsystems.test;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.PedroDrive;

public class PedroCalculate {

    private PedroDrive pedroDrive;


    //get pose of the goal and the x and y coords
    private Pose goalPoseRed = new Pose(138,138);
    private Pose goalPoseBlue = new Pose(4.5,139);
    private double goalY;
    private double goalX;

    private boolean isRed;
    //robot pose
    private Pose currentPose;
    private double robx;
    private double roby;

    private double distance;

    public PedroCalculate(HardwareMap hwMap){
        pedroDrive = new PedroDrive(hwMap);
    }

    public void getRobotPose(){
        currentPose = pedroDrive.getPose();
        robx = currentPose.getX();
        roby = currentPose.getY();
    }

    public double getPoseX(){
        return currentPose.getX();
    }
    public double getPoseY(){
        return currentPose.getY();
    }
    public void setGoal(){
        if(isRed = true){
            goalY = goalPoseRed.getY();
            goalX = goalPoseRed.getX();
        } else {
            goalY = goalPoseBlue.getY();
            goalX = goalPoseBlue.getX();
        }
    }

    public void setRed(){
        isRed = true;
    }

    public void setBlue(){
        isRed=false;
    }


    public double calcVel() {
        //dist formula for distance
        double deltaX = goalX - robx;
        double deltaY = goalY - roby;
        distance = Math.hypot(deltaX,deltaY);

        //use desmos to find a regression line
        //usea table w distance and rpm and generate a best fit line.
        double velocity = 1.56235*Math.pow(distance,2) - 171.68518*distance + 6334.2165;

        return velocity;
    }




}
