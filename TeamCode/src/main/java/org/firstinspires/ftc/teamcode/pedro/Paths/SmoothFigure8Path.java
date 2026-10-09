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

@Autonomous(group = "Autonomous")
public class SmoothFigure8Path extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(58, 8, 90);
    private final Pose path1 = poseFactory.of(58, 8, 180);
    private final Pose path1Control1 = poseFactory.of(57.7135, 46.7889, 0);
    private final Pose path1Control2 = poseFactory.of(41.7921, 19.4703, 0);
    private final Pose path1Control3 = poseFactory.of(21.2753, 29.0891, 0);
    private final Pose path1Control4 = poseFactory.of(41.5682, 37.3756, 0);
    private final Pose path1Control5 = poseFactory.of(3.2183, 124.2994, 0);
    private final Pose path1Control6 = poseFactory.of(18.2311, 134.069, 0);
    private final Pose path1Control7 = poseFactory.of(29.7769, 124.4061, 0);
    private final Pose path1Control8 = poseFactory.of(86.691, 130.7568, 0);
    private final Pose path1Control9 = poseFactory.of(102.6116, 135.4125, 0);
    private final Pose path1Control10 = poseFactory.of(115.0819, 106.1413, 0);
    private final Pose path1Control11 = poseFactory.of(42.3692, 63.2496, 0);
    private final Pose path1Control12 = poseFactory.of(33.7319, 24.7544, 0);
    private final Pose path1Control13 = poseFactory.of(57.4799, 4.13, 0);
    private final Pose path1Control14 = poseFactory.of(51.2528, 2.1998, 0);
    private final Pose path1Control15 = poseFactory.of(119.8094, 2.3068, 0);
    private final Pose path1Control16 = poseFactory.of(121.9302, 14.7448, 0);
    private final Pose path1Control17 = poseFactory.of(132.1509, 32.3949, 0);
    private final Pose path1Control18 = poseFactory.of(138.5859, 99.4543, 0);
    private final Pose path1Control19 = poseFactory.of(127.3242, 132.1661, 0);
    private final Pose path1Control20 = poseFactory.of(86.8547, 141.012, 0);
    private final Pose path1Control21 = poseFactory.of(76.6059, 141.6774, 0);
    private final Pose path1Control22 = poseFactory.of(67.8957, 136.8162, 0);
    private final Pose path1Control23 = poseFactory.of(36.0096, 79.5457, 0);
    private final Pose path1Control24 = poseFactory.of(93.0249, 72.4414, 0);
    private final Pose path1Control25 = poseFactory.of(75.3724, 46.4165, 0);
    private final Pose path1Control26 = poseFactory.of(68.9213, 36.9848, 0);
    private final Pose path1Control27 = poseFactory.of(56.1926, 34.5778, 0);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
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
    }

    public Path path1() {
        return Paths.curve(start, path1Control1, path1Control2, path1Control3, path1Control4, path1Control5, path1Control6, path1Control7, path1Control8, path1Control9, path1Control10, path1Control11, path1Control12, path1Control13, path1Control14, path1Control15, path1Control16, path1Control17, path1Control18, path1Control19, path1Control20, path1Control21, path1Control22, path1Control23, path1Control24, path1Control25, path1Control26, path1Control27, path1).linear(start, path1);
    }
}
