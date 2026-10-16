package org.firstinspires.ftc.teamcode.BIOBUZZouttakeTest;

import static com.pedropathing.api.Paths.*;
import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.pedropathing.paths.interpolator.Interpolator;

import org.firstinspires.ftc.teamcode.pedro.Constants.Constants;

@Autonomous(name = "AutoPathA", group = "Autonomous")
public class AutonPathA extends LinearOpMode {


    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(58.1972, 9.538, 90);
    private final Pose point1 = poseFactory.of(8.679, 10.8098, 270);
    private final Pose point1Control1 = poseFactory.of(10.4728, 33.5233, 0);
    private final Pose point1Segment1Start = poseFactory.of(8.679, 10.8098, 90);
    private final Pose point1Segment1End = poseFactory.of(8.679, 10.8098, 270);
    private final Pose point1Segment2Start = poseFactory.of(8.679, 10.8098, 270);
    private final Pose point1Segment2End = poseFactory.of(8.679, 10.8098, 270);
    private final Pose point2Start = poseFactory.of(8.679, 10.8098, 270);
    private final Pose point2 = poseFactory.of(58.5555, 132.5349, 270);
    private final Pose point2Control1 = poseFactory.of(83.0481, 3.8447, 0);
    private final Pose point2Control2 = poseFactory.of(63.8393, 92.9526, 0);
    private final Pose point3 = poseFactory.of(9.5578, 114.9573, 0);
    private final Pose point3Control1 = poseFactory.of(68.2232, 106.8276, 0);
    private final Pose point3Segment1Start = poseFactory.of(9.5578, 114.9573, 270);
    private final Pose point3Segment1End = poseFactory.of(9.5578, 114.9573, 0);
    private final Pose point3Segment2Start = poseFactory.of(9.5578, 114.9573, 0);
    private final Pose point3Segment2End = poseFactory.of(9.5578, 114.9573, 0);

    // Autonomous routine
    public Command autoRoutine() {

        Command w1 = waitMs(1000);
        Command w2 = waitMs(2000);
        Command w3 = waitMs(3000);
        Command w4 = waitMs(4000);
        Command w5 = waitMs(5000);


        return sequential(

                w4,
                follow(follower, path1()),
                w2,
                follow(follower, path2()),
                w5,
                follow(follower, path3())
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }

            telemetry.update();
        }
    }


    public Path path1() {
        return Paths.curve(start, point1Control1, point1).heading(Interpolator.piecewise().until(0.5843, Interpolator.linear(point1Segment1Start, point1Segment1End)).until(1, Interpolator.linear(point1Segment2Start, point1Segment2End)));
    }

    public Path path2() {
        return Paths.curve(point2Start, point2Control1, point2Control2, point2).linear(point2Start, point2);
    }

    public Path path3() {
        return Paths.curve(point2, point3Control1, point3).heading(Interpolator.piecewise().until(0.9, Interpolator.linear(point3Segment1Start, point3Segment1End)).until(1, Interpolator.linear(point3Segment2Start, point3Segment2End)));
    }
}