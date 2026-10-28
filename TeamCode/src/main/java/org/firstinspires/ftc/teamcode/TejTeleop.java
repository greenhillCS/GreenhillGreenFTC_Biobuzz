package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedro.Constants;


@TeleOp(name="TejTeleop", group="Linear OpMode")

public class TejTeleop extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;
    private Servo gate = null;
    private DcMotor intake = null;
    private DcMotorEx shooter = null;
    private Follower follower;


    int shooterPower = 2090;
    double intakePower = 1.0;
    boolean shooterOn = true;
    boolean intakeOn = false;


    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        follower = Constants.create(hardwareMap);
        intake = hardwareMap.get(DcMotor.class, "intake");
        shooter = hardwareMap.get(DcMotorEx.class,"shooter");
        gate = hardwareMap.get(Servo.class, "gate");
        gate.setPosition(0);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if (intakeOn){
                intake.setPower(intakePower);
            } else {
                intake.setPower(0);
            }

            if(gamepad1.right_trigger_pressed){
                intakeOn = true;
            }else{
                intakeOn = false;
            }

            if (gamepad1.left_trigger_pressed){
                intakePower = -1.0;
            }else{
                intakePower = 1.0;
            }

            if(shooterOn) {
                shooter.setVelocity(shooterPower);
            } else {
                shooter.setVelocity(0);
            }

            if (gamepad1.yWasPressed()){
                shooterOn = !shooterOn;
            }

            if (gamepad1.leftBumperWasPressed()) {
                shooterPower -= 10;
            }

            if (gamepad1.rightBumperWasPressed()) {
                shooterPower += 10;
            }


            if(gamepad1.b) {
                gate.setPosition(0.8);
            }else{
                gate.setPosition(0);
            }

            ManualDrive.driveOrHold(
                    follower,
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x
            );
            follower.update();

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Velocity", shooterPower);
            telemetry.update();
        }
    }}
