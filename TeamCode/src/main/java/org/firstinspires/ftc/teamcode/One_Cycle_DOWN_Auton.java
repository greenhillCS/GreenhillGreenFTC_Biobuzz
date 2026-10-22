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

@Autonomous(name = "One_Cycle_DOWN_Auton", group = "Autonomous")
public class One_Cycle_DOWN_Auton extends LinearOpMode {
    private Follower follower;

    // Hardware components for mechanisms
    private Servo gate = null;
    private DcMotor intake = null;
    private DcMotorEx shooter = null;

    //Shooter constants
    private int shooterVelocity = 1490;
    private int PreShootInterval = 300;
    private int PostShootInterval = 150;

    private int waitTime = 10000;

    // State tracking for telemetry
    private String robotState = "IDLE";

    //PedroPathing Poses
    private final PoseFactory poseFactory = PoseFactory.degrees();


    private final Pose start = poseFactory.of(61.3242, 131.9962, 90);
    private final Pose path1 = poseFactory.of(59.1798, 34.4101, 90);
    private final Pose point2 = poseFactory.of(12.6565, 91.7063, 90);
    private final Pose point2Control1 = poseFactory.of(19.2657, 34.256, 0);

    // Mechanism state helper methods with state tracking
    private void setIdle() {
        robotState = "IDLE";
        gate.setPosition(0);
        intake.setPower(0.0);
        shooter.setVelocity(shooterVelocity);
    }
    private void setIntake() {
        robotState = "INTAKE";
        gate.setPosition(0);
        intake.setPower(1.0);
        shooter.setVelocity(shooterVelocity);
    }
    private void setShoot() {
        robotState = "SHOOT";
        gate.setPosition(1);
        waitMs(50);
        gate.setPosition(0);
        waitMs(50);

        gate.setPosition(1);
        waitMs(50);
        gate.setPosition(0);
        waitMs(50);

        gate.setPosition(1);
        waitMs(50);
        gate.setPosition(0);
        waitMs(50);

        gate.setPosition(1);
        waitMs(50);
        gate.setPosition(0);
        waitMs(50);

        gate.setPosition(1);
        waitMs(50);
        gate.setPosition(0);
        waitMs(50);

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
                // 1. Shoot then Follow Path 1 in Idle
                // instant(this::setShoot),
                //   waitMs(shootTime),
                //    instant(this::setIdle),
                instant(() -> shooter.setVelocity(shooterVelocity)),
                waitMs(waitTime),
                instant(() -> gate.setPosition(1)),
                waitMs(PostShootInterval),
                instant(() -> gate.setPosition(0)),

                waitMs(PreShootInterval),
                instant(() -> gate.setPosition(1)),
                waitMs(PostShootInterval),
                instant(() -> gate.setPosition(0)),

                waitMs(PreShootInterval),
                instant(() -> gate.setPosition(1)),
                waitMs(PostShootInterval),
                instant(() -> gate.setPosition(0)),

                waitMs(PreShootInterval),
                instant(() -> gate.setPosition(1)),
                waitMs(PostShootInterval),
                instant(() -> gate.setPosition(0)),

                waitMs(PreShootInterval),
                instant(() -> gate.setPosition(1)),
                waitMs(PostShootInterval),
                instant(() -> gate.setPosition(0)),

                //Follow path 1 until parked
                waitMs(500),
                follow(follower, path1()),
                follow(follower, path2()),
                //Turn off everything once the final path completes
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
        return Paths.line(start, path1).constant(path1);
    }

    public Path path2() {
        return Paths.curve(path1, point2Control1, point2).constant(point2);
    }
}