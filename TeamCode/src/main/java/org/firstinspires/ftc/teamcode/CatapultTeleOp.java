package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Catapult control for a goBILDA 5203 Yellow Jacket 312 RPM motor.
 *
 * Controls (gamepad 1):
 *   A = fire (swing 0 -> 45 degrees at full power, then return to 0)
 *   B = re-zero the encoder (use only when the arm is resting at its start position)
 */
@TeleOp(name = "Catapult TeleOp", group = "Catapult")
public class CatapultTeleOp extends LinearOpMode {

    // ---------------- Tuning constants ----------------
    // goBILDA 312 RPM motor: 537.7 encoder ticks per output-shaft revolution
    static final double TICKS_PER_MOTOR_REV = 537.7;

    // If the arm is driven through gears/sprockets, set this to (arm teeth / motor teeth).
    // Direct drive = 1.0
    static final double GEAR_RATIO = 1.0;

    static final double LAUNCH_DEGREES = 15.0;   // how far the arm swings
    static final double FIRE_POWER     = 1.0;    // full power for a fast throw
    static final double RETURN_POWER   = 0.1;    // gentle return to the start
    static final int    BRAKE_EARLY_TICKS = 0;   // raise this if the arm overshoots 45 degrees
    static final double HOLD_AT_TOP_SEC   = 0.25; // pause at the top before returning
    static final double FIRE_TIMEOUT_SEC  = 0.75; // safety: give up if the arm jams

    private enum State { READY, FIRING, HOLDING, RETURNING }

    private DcMotorEx catapult;
    private State state = State.READY;
    private final ElapsedTime timer = new ElapsedTime();
    private int launchTicks;

    @Override
    public void runOpMode() {
        catapult = hardwareMap.get(DcMotorEx.class, "catapult");

        // If the arm swings the wrong way, change this to REVERSE.
        catapult.setDirection(DcMotorSimple.Direction.FORWARD);
        catapult.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Arm must be at its 0-degree rest position when the OpMode is initialized.
        catapult.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        catapult.setTargetPosition(0);
        catapult.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        launchTicks = degreesToTicks(LAUNCH_DEGREES);

        telemetry.addData("Status", "Initialized. Arm should be at rest.");
        telemetry.addData("Launch target (ticks)", launchTicks);
        telemetry.update();

        waitForStart();

        boolean lastA = false;

        while (opModeIsActive()) {
            boolean aPressed = gamepad1.a && !lastA;  // only react on the press, not hold
            lastA = gamepad1.a;

            switch (state) {
                case READY:
                    if (aPressed) {
                        startFire();
                    } else if (gamepad1.b) {
                        rezero();
                    }
                    break;

                case FIRING:
                    // Full power until we reach 45 degrees, then brake hard.
                    if (catapult.getCurrentPosition() >= launchTicks - BRAKE_EARLY_TICKS) {
                        catapult.setPower(0);   // BRAKE mode stops the arm quickly
                        state = State.HOLDING;
                        timer.reset();
                    } else if (timer.seconds() > FIRE_TIMEOUT_SEC) {
                        catapult.setPower(0);   // something is stuck; stop and come back
                        startReturn();
                    }
                    break;

                case HOLDING:
                    if (timer.seconds() > HOLD_AT_TOP_SEC) {
                        startReturn();
                    }
                    break;

                case RETURNING:
                    if (!catapult.isBusy()) {
                        state = State.READY;   // RUN_TO_POSITION keeps holding it at 0
                    }
                    break;
            }

            telemetry.addData("State", state);
            telemetry.addData("Position (ticks)", catapult.getCurrentPosition());
            telemetry.addData("Angle (deg)", "%.1f", ticksToDegrees(catapult.getCurrentPosition()));
            telemetry.addData("Velocity (ticks/s)", "%.0f", catapult.getVelocity());
            telemetry.update();
        }

        catapult.setPower(0);
    }

    private void startFire() {
        // RUN_WITHOUT_ENCODER gives raw full power (no speed limiting / slowdown near target).
        // The encoder still counts, so we can still watch the position.
        catapult.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        catapult.setPower(FIRE_POWER);
        state = State.FIRING;
        timer.reset();
    }

    private void startReturn() {
        catapult.setTargetPosition(0);
        catapult.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        catapult.setPower(RETURN_POWER);
        state = State.RETURNING;
    }

    private void rezero() {
        catapult.setPower(0);
        catapult.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        catapult.setTargetPosition(0);
        catapult.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private int degreesToTicks(double degrees) {
        return (int) Math.round(degrees / 360.0 * TICKS_PER_MOTOR_REV * GEAR_RATIO);
    }

    private double ticksToDegrees(int ticks) {
        return ticks / (TICKS_PER_MOTOR_REV * GEAR_RATIO) * 360.0;
    }
}