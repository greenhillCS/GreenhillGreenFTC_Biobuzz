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

@Autonomous(name = "AutoPathA", group = "Autonomous")
public class AutonPathA extends LinearOpMode {

    private BIOBUZZToolClass tools;
    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(58.4744, 9.9775, 90);
    private final Pose point1 = poseFactory.of(55.9189, 100.8952, 0);
    private final Pose point1Control1 = poseFactory.of(71.1491, 100.4115, 0);
    private final Pose point2Start = poseFactory.of(55.9189, 100.8952, 0);
    private final Pose point2 = poseFactory.of(9.5578, 98.698, 0);
    private final Pose point2Control1 = poseFactory.of(32.8106, 104.6793, 0);

    // Autonomous routine
    public Command autoRoutine() {

        Command w1 = waitMs(1000);
        Command w2 = waitMs(2000);
        Command w3 = waitMs(3000);
        Command w4 = waitMs(4000);
        Command w5 = waitMs(5000);


        return sequential(

                parallel(
                        instant (()->tools.start()),
                        w2
                ),
                parallel(instant(()->tools.shootIn()),
                        w2
                ),

                instant(() -> tools.shootStop()),

                follow(follower, path1()),

                follow(follower, path2())
        );
    }

    @Override
    public void runOpMode() {
        tools = new BIOBUZZToolClass(hardwareMap, gamepad1, telemetry);
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
        return Paths.curve(start, point1Control1, point1).linear(start, point1);
    }

    public Path path2() {
        return Paths.curve(point2Start, point2Control1, point2).linear(point2Start, point2);
    }
}