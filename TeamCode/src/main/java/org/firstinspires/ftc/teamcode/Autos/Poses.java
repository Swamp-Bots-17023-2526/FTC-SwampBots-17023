package org.firstinspires.ftc.teamcode.Autos;


import com.pedropathing.geometry.Pose;

public class Poses {
    private final Pose startRedClose = new Pose(118, 118, Math.toRadians(225));//right up against goal, facing away
    private final Pose startRedFar = new Pose(86, 9, Math.toRadians(90)); //on line, facing forward, middle of square
    private final Pose shootPoseFarRed = new Pose(84, 84, Math.toRadians(45)); //CHANGE ONCE VELOCITY DEFINED
    private final Pose shootPoseCloseRed = new Pose(84, 84, Math.toRadians(45)); //CHANGE ONCE VELOCITY DEFINED




    public Pose mirrorPose(Pose pose){
        Pose mirror = new Pose(10,90,90);
        return mirror;
    }
}
