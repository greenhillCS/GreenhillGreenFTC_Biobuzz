package org.firstinspires.ftc.teamcode.KiranaCode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class Kirana_Toolclass {
    private DcMotorEx intake = null;
    private DcMotorEx shooter = null;
    private Servo gate = null;
    Kirana_Toolclass(){
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setDirection(DcMotorEx.Direction.FORWARD);
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        gate = hardwareMap.get(Servo.class, "gate");


    }
    public void intakeOn(){
        intake.setPower(1);
    }
    public void intakeOff(){
        intake.setPower(0);
    }
    public void intakeReverse(){
        intake.setPower(-1);
    }
    public void shooterOn(int velocity) { shooter.setVelocity(velocity); }
    public void gateOpen() {gate.setPosition(1); }
    public void gateClose() {gate.setPosition(0); }
}
