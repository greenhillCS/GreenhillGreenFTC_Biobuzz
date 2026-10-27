package org.firstinspires.ftc.teamcode;
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
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "Two_Cycle_UP_Auton", group = "Autonomous")
public class Two_Cycle_UP_Auton extends LinearOpMode {
    private Follower follower;

    // Hardware components for mechanisms
    private Servo gate = null;
    private DcMotor intake = null;
    private DcMotorEx shooter = null;

    //Shooter constants
    private int shooterVelocity = 2080;
    private int PreShootInterval = 300;
    private int PostShootInterval = 150;

    // State tracking for telemetry
    private String robotState = "IDLE";

    //PedroPathing Poses
    private final PoseFactory poseFactory = PoseFactory.degrees();


    private final Pose start = poseFactory.of(52.7319, 9.7271, 270);
    private final Pose point1 = poseFactory.of(24.2271, 9.2729, 180);
    private final Pose point1Control1 = poseFactory.of(49.7408, 20.0226, 0);
    private final Pose point2 = poseFactory.of(9.2729, 9.2729, 180);
    private final Pose point3Start = poseFactory.of(9.2729, 9.2729, 180);
    private final Pose point3 = poseFactory.of(61, 120, 90);
    private final Pose point3Control1 = poseFactory.of(40.2087, 26.4583, 0);
    private final Pose point3Control2 = poseFactory.of(10.1364, 121.3555, 0);
    private final Pose point4 = poseFactory.of(61, 131.5, 90);
    private final Pose point5Start = poseFactory.of(61, 131.5, 90);
    private final Pose point5 = poseFactory.of(10.553, 121.2961, 0);
    private final Pose point5Control1 = poseFactory.of(61.0915, 110.2994, 0);
    private final Pose point5Control2 = poseFactory.of(25.1581, 120.5923, 0);
    private final Pose point5Control3 = poseFactory.of(34.5963, 115.5136, 0);

    // Mechanism state helper methods with state tracking
    private void setIdle() {
        robotState = "IDLE";
        gate.setPosition(0);
        intake.setPower(0.0);
        shooter.setVelocity(shooterVelocity);
    }
    private void setComplete() {
        robotState = "Complete";
        gate.setPosition(0);
        intake.setPower(0.0);
        shooter.setPower(0.0);
    }

    // Autonomous routine using Ivy Command Groups matching your requirements
    public Command autoRoutine() {
        return sequential(
                // SHOOT then Follow Path 1
                instant(() -> shooter.setVelocity(shooterVelocity)),
                waitMs(5000),

                instant(() -> gate.setPosition(1)),
                waitMs(2000),
                instant(() -> gate.setPosition(0)),

                // Follow Path 2 while INTAKE is active
                waitMs(500),
                instant(() -> intake.setPower(1)),
                follow(follower, path1()),
                follow(follower, path2()),

                // Follow Path 3 and Path 4
                waitMs(1000),
                follow(follower, path3()),
                follow(follower, path4()),

                // SHOOT again
                instant(() -> shooter.setVelocity(shooterVelocity)),
                waitMs(2000),

                instant(() -> gate.setPosition(1)),
                waitMs(2000),
                instant(() -> gate.setPosition(0)),

                waitMs(500),

                // Follow Path 5
                instant(this::setIdle),
                follow(follower, path5()),

                // Turn off everything once the final path completes
                instant(this::setComplete)
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();

        // Hardware mapping
        intake = hardwareMap.get(DcMotor.class, "intake");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        gate = hardwareMap.get(Servo.class, "gate");

        intake.setDirection(DcMotor.Direction.FORWARD);
        shooter.setDirection(DcMotor.Direction.FORWARD);

        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        gate.setPosition(0);

        waitForStart();

        Scheduler.reset();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();

            // Telemetry updates
            telemetry.addData("Robot State", robotState);
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
        return Paths.curve(start, point1Control1, point1).linear(start, point1);
    }

    public Path path2() {
        return Paths.line(point1, point2).tangent();
    }

    public Path path3() {
        return Paths.curve(point3Start, point3Control1, point3Control2, point3).linear(point3Start, point3);
    }

    public Path path4() {
        return Paths.line(point3, point4).constant(point4);
    }

    public Path path5() {
        return Paths.curve(point5Start, point5Control1, point5Control2, point5Control3, point5).linear(point5Start, point5);
    }
}