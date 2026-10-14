package org.firstinspires.ftc.teamcode.BIOBUZZouttakeTest;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class BIOBUZZToolClass {
    DcMotorEx shooter;
    DcMotor gate;
    public BIOBUZZToolClass(HardwareMap hardwareMap){
        shooter = hardwareMap.get(DcMotorEx.class, "outtake");
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        gate = hardwareMap.get(DcMotor.class, "transfer_motor");
    }
}
