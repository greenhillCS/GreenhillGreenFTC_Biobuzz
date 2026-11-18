package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.Position_Storage;

@TeleOp(group="Test")
public class TestNoPedroDrivingTeleOp extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private Follower follower;

    private Pose robotPose;

    private TejToolClass tools;
    private TejPathClass path;
    private NoPedroDrivingClass drive;
    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        follower = Constants.create(hardwareMap);
        follower.setPose(Position_Storage.pos);
        follower.update();

        tools = new TejToolClass(hardwareMap, gamepad1, telemetry);
        path = new TejPathClass(follower, gamepad1);
        drive = new NoPedroDrivingClass(hardwareMap, gamepad1, telemetry);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            Position_Storage.pos = new Pose((double) follower.pose().x(), (double) follower.pose().y(), (double) follower.pose().heading());


            tools.update1();

//            ManualDrive.driveOrHold(
//                    follower,
//                    -gamepad1.left_stick_y,
//                    -gamepad1.left_stick_x,
//                    -gamepad1.right_stick_x
//            );

            if(!drive.update()){
                follower.hold(follower.pose());
            }

            path.update();

            follower.update();

            robotPose = follower.pose();

            telemetry.addData("Robot X", robotPose.x());
            telemetry.addData("Robot Y", robotPose.y());
            telemetry.addData("Robot Heading", Math.toDegrees(robotPose.heading()));
            telemetry.addData("Status", "Run Time: " + runtime.toString());

            telemetry.update();
        }
    }
}
