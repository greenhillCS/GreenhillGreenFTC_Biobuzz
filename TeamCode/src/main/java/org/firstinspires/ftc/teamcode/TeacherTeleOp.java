package org.firstinspires.ftc.teamcode;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.commands.Commands.lazy;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.TeacherBotConstants;

@TeleOp(name = "Teacher TeleOp")
public class TeacherTeleOp extends OpMode {

    private Follower follower;
    private Command driveToShoot;

    // Your shooting spot: x, y in inches, heading in radians
    private final Pose shootPose = new Pose(56, 8, Math.toRadians(90));

    @Override
    public void init() {
        Scheduler.reset();
        follower = TeacherBotConstants.create(hardwareMap);
        follower.setPose(TeacherAuton.autonEndingPose);
        follower.update();
    }

    @Override
    public void loop() {

        boolean autoDriving = driveToShoot != null && driveToShoot.isScheduled();

        // A = auto-drive to the shooting position
        if (gamepad1.aWasPressed() && !autoDriving) {
            driveToShoot = lazy(() -> {
                Pose start = follower.pose();   // read pose when the command starts
                return follow(follower, line(start, shootPose).linear(start, shootPose));
            });
            driveToShoot.schedule();
            autoDriving = true;
        }

        // Moving the sticks or pressing B cancels auto-drive
        boolean sticksMoved = Math.abs(gamepad1.left_stick_x) > 0.1
                || Math.abs(gamepad1.left_stick_y) > 0.1
                || Math.abs(gamepad1.right_stick_x) > 0.1;

        if (autoDriving && (sticksMoved || gamepad1.bWasPressed())) {
            driveToShoot.cancel();
            follower.stop();
            autoDriving = false;
        }

        if (!autoDriving) {
            ManualDrive.driveOrHold(
                    follower,
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x
            );
        }
        Scheduler.execute();
        follower.update();
        Pose robotPose = follower.pose(); // returns a Pose object
        TeacherAuton.autonEndingPose = robotPose;
        telemetry.addData("Robot X", robotPose.x());
        telemetry.addData("Robot Y", robotPose.y());
        telemetry.addData("Robot Heading", Math.toDegrees(robotPose.heading()));
        telemetry.update();
    }
}