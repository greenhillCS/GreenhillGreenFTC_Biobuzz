package org.firstinspires.ftc.teamcode.pedro.Paths;


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

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(group = "Autonomous")
public class TestCompPath extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(59, 9.1, 90);
    private final Pose path1 = poseFactory.of(9.6814, 8.8644, -180);
    private final Pose path1Control1 = poseFactory.of(46.9149, 29.1364, 0);
    private final Pose path1Control2 = poseFactory.of(43.8555, 8.5457, 0);
    private final Pose path1Control3 = poseFactory.of(34.6942, 13.0257, 0);
    private final Pose path1Segment1Start = poseFactory.of(9.6814, 8.8644, 90);
    private final Pose path1Segment1End = poseFactory.of(9.6814, 8.8644, 190);
    private final Pose path1Segment2Start = poseFactory.of(9.6814, 8.8644, -170);
    private final Pose path1Segment2End = poseFactory.of(9.6814, 8.8644, 190);
    private final Pose path1Segment3Start = poseFactory.of(9.6814, 8.8644, -170);
    private final Pose path1Segment3End = poseFactory.of(9.6814, 8.8644, 180);
    private final Pose point2Start = poseFactory.of(9.6814, 8.8644, 180);
    private final Pose point2 = poseFactory.of(60, 120.0064, 270);
    private final Pose point2Control1 = poseFactory.of(40.8331, 31.1958, 0);
    private final Pose point2Control2 = poseFactory.of(21.6284, 67.114, 0);
    private final Pose point2Control3 = poseFactory.of(23.6469, 122.8451, 0);
    private final Pose point3 = poseFactory.of(60.2271, 132.2713, -91.0609);
    private final Pose point4Start = poseFactory.of(60.2271, 132.2713, 270);
    private final Pose point4 = poseFactory.of(9.061, 106.2737, 0);
    private final Pose point4Control1 = poseFactory.of(59.7319, 106.7665, 0);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, path1()),
                follow(follower, path2()),
                follow(follower, path3()),
                follow(follower, path4())
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
        return Paths.curve(start, path1Control1, path1Control2, path1Control3, path1).heading(Interpolator.piecewise().until(0.3054, Interpolator.linear(path1Segment1Start, path1Segment1End)).until(0.8672, Interpolator.linear(path1Segment2Start, path1Segment2End)).until(1, Interpolator.linear(path1Segment3Start, path1Segment3End)));
    }

    public Path path2() {
        return Paths.curve(point2Start, point2Control1, point2Control2, point2Control3, point2).linear(point2Start, point2);
    }

    public Path path3() {
        return Paths.line(point2, point3).reverseTangent();
    }

    public Path path4() {
        return Paths.curve(point4Start, point4Control1, point4).linear(point4Start, point4);
    }
}