package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.ExpansionHubServo;

public class Lift implements Subsystem{
    private ExpansionHubMotor winch;
    private ExpansionHubServo claw;
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
        claw = m_hardwareMap.get(ExpansionHubServo.class,"claw");
    }
    public void setPosition(){}
    public void update(){
        if (m_gamepad.a){
            goUp();
        }
        else if (m_gamepad.b){
            goDown();
        }
        else{
            stopWinch();
        }
        if (m_gamepad.right_stick_button){
            claw.setPosition(.8);
        }
        else if (m_gamepad.left_stick_button){
            claw.setPosition(.3);
        }
    }
    public void goUp(){
        winch.setPower(-.7);
    }
    public void goDown(){
        winch.setPower(.7);
    }
    public void stopWinch(){
        winch.setPower(0);
    }
}
