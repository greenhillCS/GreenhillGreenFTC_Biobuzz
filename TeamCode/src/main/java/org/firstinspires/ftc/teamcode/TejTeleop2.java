package org.firstinspires.ftc.teamcode;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp(name="TejTeleop2", group="Linear OpMode")
public class TejTeleop2 extends LinearOpMode {

    private Servo gate = null;
    private DcMotor intake = null;
    private DcMotorEx shooter = null;

    private int shooterVelocity = 2080;
    private int PreShootInterval = 300;
    private int PostShootInterval = 150;

    private ElapsedTime runtime = new ElapsedTime();



    int idleShooterVelocity = shooterVelocity;

    private ElapsedTime timer = new ElapsedTime();

    // Flags to handle gamepad button taps (Edge Detection)
    private Follower follower;

    private enum RobotState {
        INTAKE,
        SHOOT,
        REVERSE,
        OFF
    }

    // Simplified into two clean alternating loop states
    private enum ServoState {
        GATE_CLOSED,
        GATE_OPEN
    }
    private ServoState currentServoState = ServoState.GATE_CLOSED;
    private RobotState currentRobotState = RobotState.INTAKE;

    @Override
    public void runOpMode() {

        follower = Constants.create(hardwareMap);

        intake.setDirection(DcMotor.Direction.FORWARD);
        shooter.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();
        timer.reset(); // Make sure our loop timer starts clean

        while (opModeIsActive()) {

            // --- State Switching using Dpad ---
            if (gamepad1.dpadUpWasPressed()) {
                currentRobotState = RobotState.SHOOT;
                timer.reset(); // Reset timer right when switching to shoot!
                currentServoState = ServoState.GATE_CLOSED;
            } else if (gamepad1.dpadDownWasPressed()) {
                currentRobotState = RobotState.INTAKE;
            } else if (gamepad1.dpadLeftWasPressed()) {
                currentRobotState = RobotState.REVERSE;
            } else if (gamepad1.dpadRightWasPressed()) {
                currentRobotState = RobotState.OFF;
            }

            // --- State Machine Behavior ---
            switch (currentRobotState) {
                case INTAKE:
                    gate.setPosition(0.0);      // Keep gate closed
                    intake.setPower(1.0);       // Run intake
                    shooter.setVelocity(shooterVelocity);
                    break;

                case SHOOT:
                    shooter.setVelocity(shooterVelocity);
                    intake.setPower(0.0); // Shut off intake during shooting

                    //250ms alternation loop
                    switch (currentServoState) {
                        case GATE_CLOSED:
                            gate.setPosition(0.0);
                            if (timer.milliseconds() >= PreShootInterval) {
                                gate.setPosition(0.8); // Flips open
                                timer.reset();
                                currentServoState = ServoState.GATE_OPEN;
                            }
                            break;

                        case GATE_OPEN:
                            gate.setPosition(0.8);
                            if (timer.milliseconds() >= PostShootInterval) {
                                gate.setPosition(0.0); // Flips closed
                                timer.reset();
                                currentServoState = ServoState.GATE_CLOSED;
                            }
                            break;
                    }
                    break;

                case REVERSE:
                    gate.setPosition(0.0);       // Keep gate closed
                    intake.setPower(-1.0);       // Run intake in Reverse
                    break;

                case OFF:
                    gate.setPosition(0);         // Keep gate closed
                    intake.setPower(0);          // Intake Off
                    shooter.setVelocity(0);      // Shooter off
                    break;
            }

            // Change shoot speed with Bumpers/Triggers
            if (gamepad1.leftBumperWasPressed()) {
                shooterVelocity -= 10;
            }

            if (gamepad1.rightBumperWasPressed()) {
                shooterVelocity += 10;
            }
            if (gamepad1.leftTriggerWasPressed()) {
                shooterVelocity -= 100;
            }

            if (gamepad1.rightTriggerWasPressed()) {
                shooterVelocity += 100;
            }

            // Send calculated power to wheels
            ManualDrive.driveOrHold(
                    follower,
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x
            );
            follower.update();

            // Show telemetry data
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Robot State", currentRobotState);
            telemetry.addData("Servo State", currentServoState);
            telemetry.addData("Shooter Velocity Target", shooterVelocity);
            telemetry.update();
        }
    }
}
