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

@Autonomous(name = "One_Cycle_UP_Auton", group = "Autonomous")
public class One_Cycle_UP_Auton extends LinearOpMode {
    private Follower follower;

    // Hardware components for mechanisms
    private Servo gate = null;
    private DcMotor intake = null;
    private DcMotorEx shooter = null;

    //Shooter constants
    private int shooterVelocity = 1550;
    private int shootInterval = 400;

    // State tracking for telemetry
    private String robotState = "IDLE";

    //PedroPathing Poses
    private final PoseFactory poseFactory = PoseFactory.degrees();


    private final Pose start = poseFactory.of(57.817, 10.2713, 270);
    private final Pose path1 = poseFactory.of(11.7103, 90.7376, 0);
    private final Pose path1Control1 = poseFactory.of(22.7986, 40.7231, 0);

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
                waitMs(3000),
                instant(() -> gate.setPosition(0.8)),
                waitMs(shootInterval),
                instant(() -> gate.setPosition(0)),

                waitMs(shootInterval),
                instant(() -> gate.setPosition(0.8)),
                waitMs(shootInterval),
                instant(() -> gate.setPosition(0)),

                waitMs(shootInterval),
                instant(() -> gate.setPosition(0.8)),
                waitMs(shootInterval),
                instant(() -> gate.setPosition(0)),

                waitMs(shootInterval),
                instant(() -> gate.setPosition(0.8)),
                waitMs(shootInterval),
                instant(() -> gate.setPosition(0)),

                //Follow path 1 until parked
                waitMs(500),
                follow(follower, path1()),

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
        return Paths.curve(start, path1Control1, path1).linear(start, path1);
    }



}