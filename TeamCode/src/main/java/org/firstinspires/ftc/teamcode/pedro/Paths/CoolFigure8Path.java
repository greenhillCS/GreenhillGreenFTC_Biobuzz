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

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "AutoPath", group = "Autonomous")
public class CoolFigure8Path extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(60, 9, 90);
    private final Pose path1 = poseFactory.of(54.624, 35.013, 180);
    private final Pose path1Control1 = poseFactory.of(60.7921, 37.114, 0);
    private final Pose point2Start = poseFactory.of(54.624, 35.013, 180);
    private final Pose point2 = poseFactory.of(54.4101, 104.9839, 0);
    private final Pose point2Control1 = poseFactory.of(24.4703, 35.8299, 0);
    private final Pose point2Control2 = poseFactory.of(27.6629, 70.5144, 0);
    private final Pose point2Control3 = poseFactory.of(24.2657, 104.8202, 0);
    private final Pose point3Start = poseFactory.of(54.4101, 104.9839, 0);
    private final Pose point3 = poseFactory.of(83.7729, 34.7103, 0);
    private final Pose point3Control1 = poseFactory.of(85.4759, 109.8387, 0);
    private final Pose point3Control2 = poseFactory.of(56.423, 33.1043, 0);
    private final Pose point4Start = poseFactory.of(83.7729, 34.7103, 0);
    private final Pose point4 = poseFactory.of(83.2584, 105.4631, 180);
    private final Pose point4Control1 = poseFactory.of(115.3467, 30.7921, 0);
    private final Pose point4Control2 = poseFactory.of(115.309, 70.1669, 0);
    private final Pose point4Control3 = poseFactory.of(115.492, 107.5923, 0);
    private final Pose point5Start = poseFactory.of(83.2584, 105.4631, 180);
    private final Pose point5 = poseFactory.of(64.0538, 34.3002, 180);
    private final Pose point5Control1 = poseFactory.of(60.8708, 103.6902, 0);
    private final Pose point5Control2 = poseFactory.of(78.5803, 36.6372, 0);
    private final Pose point6Start = poseFactory.of(64.0538, 34.3002, 180);
    private final Pose point6 = poseFactory.of(60, 9, 90);
    private final Pose point6Control1 = poseFactory.of(59.6404, 34.9061, 0);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, path1()),
                follow(follower, path2()),
                follow(follower, path3()),
                follow(follower, path4()),
                follow(follower, path5()),
                follow(follower, path6())
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
        return Paths.curve(start, path1Control1, path1).linear(start, path1);
    }

    public Path path2() {
        return Paths.curve(point2Start, point2Control1, point2Control2, point2Control3, point2).linear(point2Start, point2);
    }

    public Path path3() {
        return Paths.curve(point3Start, point3Control1, point3Control2, point3).linear(point3Start, point3);
    }

    public Path path4() {
        return Paths.curve(point4Start, point4Control1, point4Control2, point4Control3, point4).linear(point4Start, point4);
    }

    public Path path5() {
        return Paths.curve(point5Start, point5Control1, point5Control2, point5).linear(point5Start, point5);
    }

    public Path path6() {
        return Paths.curve(point6Start, point6Control1, point6).linear(point6Start, point6);
    }
}