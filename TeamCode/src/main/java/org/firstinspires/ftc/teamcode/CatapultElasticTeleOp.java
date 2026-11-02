package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Elastic-powered catapult with a goBILDA 312 RPM motor.
 *
 * HOW IT WORKS
 *   Elastic bands pull the arm FORWARD against a hard stop (the launch position).
 *   The motor pulls the arm BACK (cocks it) by SWING_DEGREES, stretching the bands.
 *   Firing releases the motor, the bands snap the arm forward, and the arm stops
 *   at the hard stop, launching the balls. Then the motor re-cocks automatically.
 *
 * ENCODER ZERO
 *   0 ticks = arm resting forward against the hard stop (where the bands pull it).
 *   Positive ticks = arm pulled back (cocked).
 *   Make sure the arm is resting against the stop when you press INIT.
 *
 * CONTROLS (gamepad 1)
 *   A = cock (when uncocked) / fire (when cocked)
 *   X = uncock slowly (do this before stopping the OpMode!)
 *   B = re-zero encoder (only when uncocked and resting on the stop)
 */
@TeleOp(name = "Catapult Elastic TeleOp", group = "Catapult")
public class CatapultElasticTeleOp extends LinearOpMode {

    // ---------------- Motor / geometry ----------------
    static final double TICKS_PER_MOTOR_REV = 537.7; // goBILDA 312 RPM
    static final double GEAR_RATIO = 1.0;            // arm teeth / motor teeth (1.0 = direct drive)
    static final double SWING_DEGREES = 60.0;        // how far back the motor pulls the arm

    // ---------------- Power settings ----------------
    static final double COCK_POWER = 1.0;        // max power used to pull back against the bands
    static final double FIRE_ASSIST_POWER = 1.0; // motor pushes forward with the bands; try 0.0 to compare
    static final double UNCOCK_DEG_PER_SEC = 30.0; // how slowly X lets the arm back down

    // ---------------- Timing / safety ----------------
    static final int    BRAKE_EARLY_TICKS = 0;   // raise if the arm slams too hard past the stop
    static final double SETTLE_SEC        = 0.3; // pause after the shot before re-cocking
    static final double FIRE_TIMEOUT_SEC  = 0.5;
    static final double COCK_TIMEOUT_SEC  = 2.0; // if it can't cock in this time, bands are too strong
    static final boolean AUTO_RECOCK      = true;

    // ---------------- Optional servo latch ----------------
    // Set to true if you add a servo latch named "latch" that holds the arm cocked.
    static final boolean USE_LATCH = false;
    static final double LATCH_OPEN   = 0.0;
    static final double LATCH_CLOSED = 0.5;
    static final double LATCH_SETTLE_SEC = 0.25;

    private enum State { IDLE, COCKING, LATCHING, COCKED, FIRING, SETTLING, UNCOCKING }

    private DcMotorEx catapult;
    private Servo latch;
    private State state = State.IDLE;
    private final ElapsedTime timer = new ElapsedTime();
    private int cockTicks;
    private int uncockStartTicks;
    private String warning = "";

    @Override
    public void runOpMode() {
        catapult = hardwareMap.get(DcMotorEx.class, "catapult");
        // Positive power must pull the arm BACK (stretching the bands).
        // If it goes the wrong way, change this to REVERSE.
        catapult.setDirection(DcMotorSimple.Direction.REVERSE);
        catapult.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (USE_LATCH) {
            latch = hardwareMap.get(Servo.class, "latch");
            openLatch();
        }

        cockTicks = degreesToTicks(SWING_DEGREES);
        rezero();

        telemetry.addData("Status", "Initialized. Arm should rest on the forward stop.");
        telemetry.addData("Cock target (ticks)", cockTicks);
        telemetry.addData("Controls", "A = cock/fire, X = uncock, B = re-zero");
        telemetry.update();

        waitForStart();

        boolean lastA = false, lastX = false;

        while (opModeIsActive()) {
            boolean aPressed = gamepad1.a && !lastA;
            boolean xPressed = gamepad1.x && !lastX;
            lastA = gamepad1.a;
            lastX = gamepad1.x;

            switch (state) {
                case IDLE:
                    if (aPressed) {
                        startCock();
                    } else if (gamepad1.b) {
                        rezero();
                    }
                    break;

                case COCKING:
                    if (!catapult.isBusy()) {
                        if (USE_LATCH) {
                            closeLatch();
                            timer.reset();
                            state = State.LATCHING;
                        } else {
                            state = State.COCKED; // motor holds the arm back
                        }
                    } else if (timer.seconds() > COCK_TIMEOUT_SEC) {
                        warning = "Could not cock! Bands may be too strong. Uncocking.";
                        startUncock();
                    }
                    break;

                case LATCHING:
                    // Give the servo time to close, then let the latch take the load.
                    if (timer.seconds() > LATCH_SETTLE_SEC) {
                        catapult.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                        catapult.setPower(0);
                        state = State.COCKED;
                    }
                    break;

                case COCKED:
                    if (aPressed) {
                        fire();
                    } else if (xPressed) {
                        startUncock();
                    }
                    break;

                case FIRING:
                    if (catapult.getCurrentPosition() <= BRAKE_EARLY_TICKS) {
                        stopWithBrake();
                        timer.reset();
                        state = State.SETTLING;
                    } else if (timer.seconds() > FIRE_TIMEOUT_SEC) {
                        warning = "Fire timed out. Check for jams or weak bands.";
                        stopWithBrake();
                        timer.reset();
                        state = State.SETTLING;
                    }
                    break;

                case SETTLING:
                    if (timer.seconds() > SETTLE_SEC) {
                        if (AUTO_RECOCK) {
                            startCock();
                        } else {
                            state = State.IDLE;
                        }
                    }
                    break;

                case UNCOCKING: {
                    // Slowly move the target toward 0 so the bands can't snap the arm forward.
                    double ticksPerSec = degreesToTicks(UNCOCK_DEG_PER_SEC);
                    int target = (int) Math.round(uncockStartTicks - ticksPerSec * timer.seconds());
                    if (target < 0) target = 0;
                    catapult.setTargetPosition(target);
                    if (target == 0 && !catapult.isBusy()) {
                        stopWithBrake();
                        state = State.IDLE;
                    }
                    break;
                }
            }

            telemetry.addData("State", state);
            telemetry.addData("Position (ticks)", catapult.getCurrentPosition());
            telemetry.addData("Arm angle back (deg)", "%.1f", ticksToDegrees(catapult.getCurrentPosition()));
            telemetry.addData("Velocity (ticks/s)", "%.0f", catapult.getVelocity());
            if (!warning.isEmpty()) telemetry.addData("WARNING", warning);
            telemetry.update();
        }

        catapult.setPower(0);
    }

    private void startCock() {
        warning = "";
        if (USE_LATCH) openLatch();
        catapult.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        catapult.setTargetPosition(cockTicks);
        catapult.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        catapult.setPower(COCK_POWER);
        timer.reset();
        state = State.COCKING;
    }

    private void fire() {
        if (USE_LATCH) openLatch();
        // FLOAT + RUN_WITHOUT_ENCODER lets the arm move freely; the encoder still counts.
        catapult.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        catapult.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        catapult.setPower(-FIRE_ASSIST_POWER); // negative = forward, helping the bands
        timer.reset();
        state = State.FIRING;
    }

    private void startUncock() {
        // Motor takes the load first, then the latch opens, then the target ramps down.
        uncockStartTicks = catapult.getCurrentPosition();
        catapult.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        catapult.setTargetPosition(uncockStartTicks);
        catapult.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        catapult.setPower(1.0);
        if (USE_LATCH) openLatch();
        timer.reset();
        state = State.UNCOCKING;
    }

    private void stopWithBrake() {
        catapult.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        catapult.setPower(0);
    }

    private void rezero() {
        catapult.setPower(0);
        catapult.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        catapult.setTargetPosition(0);
        catapult.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        catapult.setPower(0);
    }

    private void openLatch()  { if (latch != null) latch.setPosition(LATCH_OPEN); }
    private void closeLatch() { if (latch != null) latch.setPosition(LATCH_CLOSED); }

    private int degreesToTicks(double degrees) {
        return (int) Math.round(degrees / 360.0 * TICKS_PER_MOTOR_REV * GEAR_RATIO);
    }

    private double ticksToDegrees(int ticks) {
        return ticks / (TICKS_PER_MOTOR_REV * GEAR_RATIO) * 360.0;
    }
}