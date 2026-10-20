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
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.pedropathing.paths.interpolator.Interpolator;

import org.firstinspires.ftc.teamcode.pedro.Constants.Constants;
import org.firstinspires.ftc.teamcode.pedro.PoseStorage;

@Autonomous(name = "AutoPathB", group = "Autonomous")
public class AutonPathB extends LinearOpMode {

    private BIOBUZZToolClass tools;

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(59.5155, 131.9224, 90);
    private final Pose path1 = poseFactory.of(9.8587, 97.302, 0);
    private final Pose path1Control1 = poseFactory.of(63.5776, 103.1918, 0);
    private final Pose path1Segment1Start = poseFactory.of(9.8587, 97.302, 270);
    private final Pose path1Segment1End = poseFactory.of(9.8587, 97.302, 270);
    private final Pose path1Segment2Start = poseFactory.of(9.8587, 97.302, 270);
    private final Pose path1Segment2End = poseFactory.of(9.8587, 97.302, 0);
    private final Pose path1Segment3Start = poseFactory.of(9.8587, 97.302, 0);
    private final Pose path1Segment3End = poseFactory.of(9.8587, 97.302, 0);

    // Autonomous routine
    public Command autoRoutine() {

        Command w1 = waitMs(1000);
        Command w2 = waitMs(2000);
        Command w3 = waitMs(3000);
        Command w4 = waitMs(4000);
        Command w5 = waitMs(5000);
        return sequential(
                w5,
                parallel(tools.shootIn(), w2),
                follow(follower, path1())
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
        PoseStorage.autonomousEndPose = follower.pose();
    }

    public Path path1() {
        return Paths.curve(start, path1Control1, path1).heading(Interpolator.piecewise().until(0.2, Interpolator.linear(path1Segment1Start, path1Segment1End)).until(0.8, Interpolator.linear(path1Segment2Start, path1Segment2End)).until(1, Interpolator.linear(path1Segment3Start, path1Segment3End)));
    }
}
