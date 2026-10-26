package org.firstinspires.ftc.teamcode;

public class TPS_RPM_Solver {

    private final double TPS = 0;
    private static final double RPM = 1625; // current RPM of the motor
    private static final double MAX_RPM = 1625;

    private void TPS_to_RPM(){
        System.out.println("Minimum RPM for " + TPS + " TPS: " + (TPS * 60));
    }
    private static void RPM_to_TPS(){
        System.out.println("Maximum TPS for " + RPM + " RPM: " + ((RPM * ((6000 / RPM) * 28)) / 60));
    }
    public static void main(String[] args){
        RPM_to_TPS();
    }
}
