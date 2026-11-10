package org.firstinspires.ftc.teamcode.KiranaCode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Kirana_Toolclass {
    private DcMotorEx intake = null;
    Kirana_Toolclass(){
        intake  = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setDirection(DcMotor.Direction.FORWARD);
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
}
