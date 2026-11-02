package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.EnumMap;
import java.util.Map;

public class PIDF {
    enum PIDs {
        Kp,
        Ki,
        Kd,
        Ks
    }
    private double integralSum = 0;
    private double lastError = 0;
    private ElapsedTime timer = new ElapsedTime();

    // Tuner stuff
    private int pidIndex = 0;
    private double[] pidInterval = {1, 0, 0.001, 0.1};
    private Map<PIDs, Double> pidVars = new EnumMap<>(PIDs.class);
    private PIDs[] pidKeys = {PIDs.Kp, PIDs.Ki, PIDs.Kd, PIDs.Ks};
    public PIDF(double Kp, double Ki, double Kd, double Ks){
        pidVars.put(PIDs.Kp, Kp);
        pidVars.put(PIDs.Ki, Ki);
        pidVars.put(PIDs.Kd, Kd);
        pidVars.put(PIDs.Ks, Ks);
    }
    public void setPID(double Kp, double Ki, double Kd){
        pidVars.put(PIDs.Kp, Kp);
        pidVars.put(PIDs.Ki, Ki);
        pidVars.put(PIDs.Kd, Kd);
    }
    public double[] getPID(){
        return new double[]{pidVars.get(PIDs.Kp), pidVars.get(PIDs.Ki), pidVars.get(PIDs.Kd), pidVars.get(PIDs.Ks)};
    }
    public void tuner(Gamepad gamepad, Telemetry telemetry){
        if(gamepad.dpadRightWasPressed()){
            pidIndex = Math.min(pidIndex + 1, pidVars.size() - 1);
        }else if(gamepad.dpadLeftWasPressed()){
            pidIndex = Math.max(pidIndex - 1, 0);
        }

        PIDs key = pidKeys[pidIndex];
        double val = pidVars.get(key);

        if(gamepad.dpadUpWasPressed()){
            pidVars.put(key, val + pidInterval[pidIndex]);
        }else if(gamepad.dpadDownWasPressed()){
            pidVars.put(key, Math.max(val - pidInterval[pidIndex], 0));
        }

        val = pidVars.get(key);

        telemetry.addData("Key", key);
        telemetry.addData("Val", val);
    }
    public double getVelo(double target, double current){
        double error = target - current;

        integralSum += (error * timer.seconds());

        double derivative = (error - lastError) / timer.seconds();

        lastError = error;
        timer.reset();

        return (pidVars.get(PIDs.Kp) * error) + (pidVars.get(PIDs.Ki) * integralSum) + (pidVars.get(PIDs.Kd) * derivative);
    }
}
