/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.BIOBUZZouttakeTest;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.Constants.Constants;
import org.firstinspires.ftc.teamcode.pedro.PoseStorage;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static com.pedropathing.api.Paths.line;

/*
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */
@TeleOp(name="BIOBUZZouttakeTest", group="Iterative OpMode")

public class BIOBUZZouttakeTest extends OpMode
{
    // Declare OpMode members.

    private ElapsedTime runtime = new ElapsedTime();
    private BIOBUZZToolClass tools;
    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();
    private final Pose topShootPos = poseFactory.of(57.8, 131.9224, 90);
    private final Pose bottomShootPos = poseFactory.of(57.8, 9.538, 90);

    private Command driveToShoot;

    enum STATE {
        TELEOP,
        AUTON
    }
    STATE state = STATE.TELEOP;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        follower = Constants.create(hardwareMap);
        tools = new BIOBUZZToolClass(hardwareMap, gamepad1, telemetry);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        Scheduler.reset();
        runtime.reset();
        follower.setPose(PoseStorage.autonomousEndPose);
        follower.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        switch (state) {
            case TELEOP:
                ManualDrive.driveOrHold(
                        follower,
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x
                );

                if (gamepad1.dpadLeftWasPressed()) {
                    Pose resetPos = new Pose(0, 0, Math.toRadians(-90));
                    follower.setPose(resetPos);
                    follower.update();
                } else if (gamepad1.dpadUpWasPressed()) {
                    driveToShoot = lazy(() -> {
                        Pose start = follower.pose(); // read pose when the command starts
                        return follow(follower, line(start, topShootPos).linear(start, topShootPos));
                    });
                    driveToShoot.schedule();
                    state = STATE.AUTON;
                }else if (gamepad1.dpadDownWasPressed()) {
                    driveToShoot = lazy(() -> {
                        Pose start = follower.pose(); // read pose when the command starts
                        return follow(follower, line(start, bottomShootPos).linear(start, bottomShootPos));
                    });
                    driveToShoot.schedule();
                    state = STATE.AUTON;
                }
            case AUTON:
                boolean sticksMoved = Math.abs(gamepad1.left_stick_x) > 0.1
                                        || Math.abs(gamepad1.left_stick_y) > 0.1
                                        || Math.abs(gamepad1.right_stick_x) > 0.1;
                if(gamepad1.dpad_right || (!follower.isBusy() && !Scheduler.isRunning(driveToShoot)) || sticksMoved){
                    follower.stop();
                    state = STATE.TELEOP;
                }
        }

        Scheduler.execute();
        follower.update();
        Pose robotPose = follower.pose(); // returns a Pose object
        telemetry.addData("Robot X", robotPose.x());
        telemetry.addData("Robot Y", robotPose.y());
        telemetry.addData("Robot Heading", Math.toDegrees(robotPose.heading()));
        // Math.toDegrees() is a built-in java method

        tools.update();
        // Show the elapsed game time and wheel power.
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        tools.stop();
    }

}
