package org.firstinspires.ftc.teamcode;

import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Disabled
@Autonomous(name = "Two_Cycle_DOWN_Auton", group = "Autonomous")
public class Two_Cycle_DOWN_Auton extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(60.5425, 132.2384, 90);
    private final Pose path1 = poseFactory.of(47.5963, 130.4848, 90);
    private final Pose path1Control1 = poseFactory.of(59.6565, 122.1541, 0);
    private final Pose path1Control2 = poseFactory.of(47.0385, 121.0369, 0);
    private final Pose point2 = poseFactory.of(60.4607, 132.2295, 90);
    private final Pose point2Control1 = poseFactory.of(46.451, 121.2673, 0);
    private final Pose point2Control2 = poseFactory.of(59.6437, 121.8892, 0);
    private final Pose point3Start = poseFactory.of(60.4607, 132.2295, 90);
    private final Pose point3 = poseFactory.of(9.8997, 121.4005, 0);
    private final Pose point3Control1 = poseFactory.of(57.0506, 117.7721, 0);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, path1()),
                follow(follower, path2()),
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
        return Paths.curve(start, path1Control1, path1Control2, path1).constant(path1);
    }

    public Path path2() {
        return Paths.curve(path1, point2Control1, point2Control2, point2).constant(point2);
    }

    public Path path3() {
        return Paths.curve(point3Start, point3Control1, point3).linear(point3Start, point3);
    }
}
