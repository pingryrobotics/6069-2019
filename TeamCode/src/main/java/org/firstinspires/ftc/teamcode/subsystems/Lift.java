package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.openftc.revextensions2.ExpansionHubMotor;

public class Lift {
    private ExpansionHubMotor winch;
    private PIDFController heightController;
    private PIDFCoefficients heighControllerCoefficients = new PIDFCoefficients();
    private double setPosition;
    private double previousPosition;
    public Lift(HardwareMap hardwareMap){
        winch = hardwareMap.get(ExpansionHubMotor.class,"winch");
        winch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        winch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);//TODO: consider swapping for run to position
    }
    public void setPosition()
    public void update()
}
