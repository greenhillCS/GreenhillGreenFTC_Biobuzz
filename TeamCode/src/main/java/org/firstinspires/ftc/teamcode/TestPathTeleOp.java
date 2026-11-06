package org.firstinspires.ftc.teamcode;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.Position_Storage;


@TeleOp(name="TestPathTeleop", group="Linear OpMode")

public class TestPathTeleOp extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private Follower follower;
    private Pose robotPose;

    enum STATES {
        DRIVING,
        SHOOTING
    }
    STATES state = STATES.DRIVING;

    private TejToolClass tools;


    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        follower = Constants.create(hardwareMap);
        follower.setPose(Position_Storage.pos);
        follower.update();

        tools = new TejToolClass(hardwareMap, gamepad1, telemetry);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            switch(follower.mode()) {
                case MANUAL:
                case IDLE:
                case HOLD:
                    tools.update1();

//                    DrivePowers powers = ManualDrive.fieldCentric(
//                            -gamepad1.left_stick_y,
//                            -gamepad1.left_stick_x,
//                            -gamepad1.right_stick_x,
//                            follower.pose().heading()
//                    );
//                    ManualDrive.driveOrHold(follower, powers);
                    ManualDrive.driveOrHold(
                            follower,
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    );

                    if (gamepad1.dpadLeftWasPressed()) {
                        follower.idle();
                        Pose corner = new Pose(9, 9, Math.toRadians(-90));
                        follower.setPose(corner);
                        follower.hold(corner);
                    } else if (gamepad1.dpadDownWasPressed()) {
                        follower.idle();
                        follower.hold(new Pose(54, 9, Math.toRadians(-90)));
                    } else if (gamepad1.dpadUpWasPressed()) {
                        follower.idle();
                        follower.hold(new Pose(66, 132, Math.toRadians(90)));
                    }

                    follower.update();
                    break;
                case FOLLOW:
                    follower.update();
                    break;
            }
            // Show the elapsed game time and wheel power.
            robotPose = follower.pose();

            telemetry.addData("Robot X", robotPose.x());
            telemetry.addData("Robot Y", robotPose.y());
            telemetry.addData("Robot Heading", Math.toDegrees(robotPose.heading()));
            telemetry.addData("Status", "Run Time: " + runtime.toString());

            telemetry.update();
        }
    }}
