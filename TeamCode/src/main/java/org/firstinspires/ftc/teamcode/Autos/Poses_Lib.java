package org.firstinspires.ftc.teamcode.Autos;


import com.pedropathing.geometry.Pose;

import java.util.ArrayList;

public class Poses_Lib {

    //Poses for red
    private final Pose startRedClose = new Pose(118, 118, Math.toRadians(225));//right up against goal, facing away
    private final Pose startRedFar = new Pose(86, 9, Math.toRadians(90)); //on line, facing forward, middle of square
    private final Pose shootPoseFarRed = new Pose(84, 84, Math.toRadians(45)); //CHANGE ONCE VELOCITY DEFINED
    private final Pose shootPoseCloseRed = new Pose(84, 84, Math.toRadians(45)); //CHANGE ONCE VELOCITY DEFINED

    //To get blue poses, mirror the red poses


    public Pose getPose(int number){
        ArrayList PoseList = new ArrayList();
        PoseList.add(startRedClose);
        PoseList.add(startRedFar);
        PoseList.add(shootPoseFarRed);
        PoseList.add(shootPoseCloseRed);
        return (Pose) PoseList.get(number);
    }



    public Pose mirrorPose(Pose input) {
        double inputX = input.getX();
        double inputY = input.getY();
        double inputHeading = input.getHeading();

        return new Pose(inputX - 72, inputY, 180 - inputHeading);
    }
}
