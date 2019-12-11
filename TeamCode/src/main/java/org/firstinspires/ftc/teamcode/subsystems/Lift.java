package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.openftc.revextensions2.ExpansionHubMotor;

public class Lift implements Subsystem{
    private ExpansionHubMotor winch;
    private PIDFController heightController;
    private PIDFCoefficients heighControllerCoefficients = new PIDFCoefficients(0,0,0,0);
    private double setPosition;
    private double previousPosition;
    private Gamepad m_gamepad;
    private HardwareMap m_hardwareMap;
    public Lift(HardwareMap hardwareMap, Gamepad gamepad){
        m_gamepad = gamepad;
        m_hardwareMap = hardwareMap;
    }
    public void initialize(){
        winch = m_hardwareMap.get(ExpansionHubMotor.class,"winch");
        winch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        winch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);//TODO: consider swapping for run to position
    }
    public void setPosition(){}
    public void update(){
        if (m_gamepad.a){
            winch.setPower(-.7);
        }
        else if (m_gamepad.b){
            winch.setPower(.7);
        }
        else{
            winch.setPower(0);
        }
    }
}
