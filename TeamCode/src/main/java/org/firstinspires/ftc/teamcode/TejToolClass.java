package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TejToolClass {
    private int shooterVelocity = 2100;
    private double intakePower = 1;
    private double gatePos = 1;
    private HardwareMap hardwareMap;
    private Gamepad gamepad;
    private Telemetry telemetry;
    private DcMotor intake;
    private DcMotorEx shooter;
    private Servo gate;

    //Update 1 Variables
    private boolean intakeReverse = false;
    private boolean shooterActive = true;

    //Update 2 Variables
    private double PreShootInterval = 300;
    private double PostShootInterval = 150;
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
    private ElapsedTime timer = new ElapsedTime();
    public TejToolClass (HardwareMap hardwareMap, Gamepad gamepad, Telemetry telemetry){
        this.hardwareMap = hardwareMap;
        this.gamepad = gamepad;
        this.telemetry = telemetry;

        intake = hardwareMap.get(DcMotor.class, "intake");
        shooter = hardwareMap.get(DcMotorEx.class,"shooter");
        gate = hardwareMap.get(Servo.class, "gate");
        gate.setPosition(0);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }
    public void shooterOn(){
        shooter.setVelocity(shooterVelocity);
    }
    public void shooterOff(){
        shooter.setVelocity(0);
    }
    public void gateOpen(){
        gate.setPosition(gatePos);
    }
    public void gateClose(){
        gate.setPosition(0);
    }
    public void intakeOn(){
        intake.setPower(intakePower);
    }
    public void intakeOn(boolean reverse){
        if (reverse) {
            intake.setPower(-intakePower);
        }else {
            intake.setPower(intakePower);
        }
    }
    public void intakeOff(){
        intake.setPower(0);
    }
    public void on(){
        intakeOn();
        gateOpen();
        shooterOn();
    }
    public void idle(){
        intakeOff();
        gateClose();
        shooterOn();
    }
    public void off(){
        intakeOff();
        gateClose();
        shooterOff();
    }
    public void update1(){

        //Intake Logic -----------------------------------------------------------------------------

        if (gamepad.left_trigger_pressed){
            intakeReverse = true;
        }else{
            intakeReverse = false;
        }

        if(gamepad.right_trigger_pressed){
            intakeOn(intakeReverse);
        }else{
            intakeOff();
        }

        //Shooter Logic ----------------------------------------------------------------------------

        if (gamepad.yWasPressed()){
            shooterActive = !shooterActive;
        }

        if(shooterActive) {
            shooterOn();
        } else {
            shooterOff();
        }

        if (gamepad.leftBumperWasPressed()) {
            shooterVelocity -= 10;
        }

        if (gamepad.rightBumperWasPressed()) {
            shooterVelocity += 10;
        }

        //Gate Logic -------------------------------------------------------------------------------

        if(gamepad.b) {
            gateOpen();
        }else{
            gateClose();
        }
        telemetry.addData("Velocity", shooterVelocity);
    }
    public void update2(){
        // --- State Switching using Dpad ---
        if (gamepad.dpadUpWasPressed()) {
            currentRobotState = RobotState.SHOOT;
            timer.reset(); // Reset timer right when switching to shoot!
            currentServoState = ServoState.GATE_CLOSED;
        } else if (gamepad.dpadDownWasPressed()) {
            currentRobotState = RobotState.INTAKE;
        } else if (gamepad.dpadLeftWasPressed()) {
            currentRobotState = RobotState.REVERSE;
        } else if (gamepad.dpadRightWasPressed()) {
            currentRobotState = RobotState.OFF;
        }

        // --- State Machine Behavior ---
        switch (currentRobotState) {
            case INTAKE:
                gateClose();      // Keep gate closed
                intakeOn();       // Run intake
                shooterOn();
                break;

            case SHOOT:
                shooterOn();
                intakeOff(); // Shut off intake during shooting

                //250ms alternation loop
                switch (currentServoState) {
                    case GATE_CLOSED:
                        gateClose();
                        if (timer.milliseconds() >= PreShootInterval) {
                            gateOpen(); // Flips open
                            timer.reset();
                            currentServoState = ServoState.GATE_OPEN;
                        }
                        break;

                    case GATE_OPEN:
                        gateOpen();
                        if (timer.milliseconds() >= PostShootInterval) {
                            gateClose(); // Flips closed
                            timer.reset();
                            currentServoState = ServoState.GATE_CLOSED;
                        }
                        break;
                }
                break;

            case REVERSE:
                gateClose();       // Keep gate closed
                intakeOn(true);       // Run intake in Reverse
                break;

            case OFF:
                gateClose();        // Keep gate closed
                intakeOff();        // Intake Off
                shooterOff();      // Shooter off
                break;
        }

        // Change shoot speed with Bumpers/Triggers
        if (gamepad.leftBumperWasPressed()) {
            shooterVelocity -= 10;
        }

        if (gamepad.rightBumperWasPressed()) {
            shooterVelocity += 10;
        }
        if (gamepad.leftTriggerWasPressed()) {
            shooterVelocity -= 100;
        }

        if (gamepad.rightTriggerWasPressed()) {
            shooterVelocity += 100;
        }


        // Show telemetry data
        telemetry.addData("Timer", timer.toString());
        telemetry.addData("Robot State", currentRobotState);
        telemetry.addData("Servo State", currentServoState);
        telemetry.addData("Shooter Velocity Target", shooterVelocity);
    }
}
