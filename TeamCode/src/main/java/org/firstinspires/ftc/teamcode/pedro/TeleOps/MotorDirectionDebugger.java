package org.firstinspires.ftc.teamcode.pedro.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * This is a simple teleop routine for debugging your motor configuration.
 * Pressing each of the buttons will power its respective motor.
 *
 * Button Mappings:
 *
 * Xbox/PS4 Button - Motor
 *   X / ▢         - Front Left
 *   Y / Δ         - Front Right
 *   B / O         - Rear  Right
 *   A / X         - Rear  Left
 *                                    The buttons are mapped to match the wheels spatially if you
 *                                    were to rotate the gamepad 45deg°. x/square is the front left
 *                    ________        and each button corresponds to the wheel as you go clockwise
 *                   / ______ \
 *     ------------.-'   _  '-..+              Front of Bot
 *              /   _  ( Y )  _  \                  ^
 *             |  ( X )  _  ( B ) |     Front Left   \    Front Right
 *        ___  '.      ( A )     /|       Wheel       \      Wheel
 *      .'    '.    '-._____.-'  .'       (x/▢)        \     (Y/Δ)
 *     |       |                 |                      \
 *      '.___.' '.               |          Rear Left    \   Rear Right
 *               '.             /             Wheel       \    Wheel
 *                \.          .'              (A/X)        \   (B/O)
 *                  \________/
 *
 * Uncomment the @Disabled tag below to use this opmode.
 */
@TeleOp(group = "drive")
public class MotorDirectionDebugger extends LinearOpMode {
    public static double MOTOR_POWER = 0.7;

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotorEx frontLeftDrive = hardwareMap.get(DcMotorEx.class, Constants.drivetrainConfig.frontLeftName.get());
        DcMotorEx frontRightDrive = hardwareMap.get(DcMotorEx.class, Constants.drivetrainConfig.frontRightName.get());
        DcMotorEx backLeftDrive = hardwareMap.get(DcMotorEx.class, Constants.drivetrainConfig.backLeftName.get());
        DcMotorEx backRightDrive = hardwareMap.get(DcMotorEx.class, Constants.drivetrainConfig.backRightName.get());

        telemetry.addLine("Press play to begin the debugging opmode");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        telemetry.clearAll();
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);

        while (!isStopRequested()) {
            telemetry.addLine("Press each button to turn on its respective motor");
            telemetry.addLine();
            telemetry.addLine("<font face=\"monospace\">Xbox/PS4 Button - Motor</font>");
            telemetry.addLine("<font face=\"monospace\">&nbsp;&nbsp;X / ▢&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;- Front Left</font>");
            telemetry.addLine("<font face=\"monospace\">&nbsp;&nbsp;Y / Δ&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;- Front Right</font>");
            telemetry.addLine("<font face=\"monospace\">&nbsp;&nbsp;B / O&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;- Rear&nbsp;&nbsp;Right</font>");
            telemetry.addLine("<font face=\"monospace\">&nbsp;&nbsp;A / X&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;- Rear&nbsp;&nbsp;Left</font>");
            telemetry.addLine();

            frontLeftDrive.setPower(0);
            frontRightDrive.setPower(0);
            backLeftDrive.setPower(0);
            backRightDrive.setPower(0);

            if(gamepad1.x) {
                frontLeftDrive.setPower(MOTOR_POWER);
                telemetry.addLine("Running Motor: Front Left");
            } else if(gamepad1.y) {
                frontRightDrive.setPower(MOTOR_POWER);
                telemetry.addLine("Running Motor: Front Right");
            } else if(gamepad1.b) {
                backRightDrive.setPower(MOTOR_POWER);
                telemetry.addLine("Running Motor: Rear Right");
            } else if(gamepad1.a) {
                backLeftDrive.setPower(MOTOR_POWER);
                telemetry.addLine("Running Motor: Rear Left");
            } else {
                telemetry.addLine("Running Motor: None");
            }

            telemetry.update();
        }
    }
}
