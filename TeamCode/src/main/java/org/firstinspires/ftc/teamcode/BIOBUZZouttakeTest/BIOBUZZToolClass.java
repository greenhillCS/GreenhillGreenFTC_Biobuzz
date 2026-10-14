package org.firstinspires.ftc.teamcode.BIOBUZZouttakeTest;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class BIOBUZZToolClass {
    DcMotorEx shooter;
    DcMotor gate;
    DcMotor intake;
    private int rpm = 1120;
    public BIOBUZZToolClass(HardwareMap hardwareMap){
        shooter = hardwareMap.get(DcMotorEx.class, "outtake");
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        gate = hardwareMap.get(DcMotor.class, "transfer_motor");
        gate.setDirection(DcMotorSimple.Direction.FORWARD);

        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
    }
    public void start(){
        //Starts the intake and shooter
        intake.setPower(0.5);
        shooter.setVelocity(rpm);
    }
    public void stop(){
        //Stops the intake and shooter
        intake.setPower(0);
        shooter.setVelocity(0);
    }
    public void shootIn(){
        //Makes the shooter shoot the staged pollen by turning the gate in
        gate.setPower(-0.5);
    }
    public void shootOut(){
        //Makes the shooter outtake the staged pollen by turning the gate out
        gate.setPower(0.5);
    }
    public void shootStop(){
        //Makes shooter stop shooting by stoping the gate
        gate.setPower(0);
    }
}
