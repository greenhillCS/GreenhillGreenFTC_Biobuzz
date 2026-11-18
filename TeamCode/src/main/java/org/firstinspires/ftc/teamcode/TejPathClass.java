package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.pedro.Constants;

public class TejPathClass {
    private Follower follower;
    private Gamepad gamepad;
    private double wallBuffer = 3; //Inches
    private Pose leftPos = new Pose(66, 126 - wallBuffer, Math.toRadians(90));
    private Pose rightPos = new Pose(55.7628, Constants.robotFrontDepth + wallBuffer, Math.toRadians(-90));
    private Pose resetPos = new Pose(Constants.robotWidth / 2, Constants.robotFrontDepth, Math.toRadians(-90));

    public TejPathClass(Follower follower, Gamepad gamepad){
        this.follower = follower;
        this.gamepad = gamepad;
    }

    public void update(){
        if (gamepad.dpadLeftWasPressed()) {
            follower.idle();
            follower.setPose(resetPos);
            follower.hold(resetPos);
        } else if (gamepad.rightBumperWasPressed()) {// Back Waypoint
            follower.idle();
            follower.hold(rightPos);
        } else if (gamepad.leftBumperWasPressed()) {// Front Waypoint
            follower.idle();
            follower.hold(leftPos);
        }
    }
}
