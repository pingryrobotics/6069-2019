package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.openftc.revextensions2.ExpansionHubServo;

public class StoneGrabber implements Subsystem {
    private ExpansionHubServo m_grabberServo;
    private final double GRAB_POSITION = .9;
    private final double RELEASE_POSITION = .1;
    private State m_state;
    private Gamepad m_gamepad;
    private HardwareMap m_hardwareMap;
    public enum State{
        GRAB,
        RELEASE
    }
    public StoneGrabber(HardwareMap hardwareMap, Gamepad gamepad){
        m_gamepad = gamepad;
        m_hardwareMap = hardwareMap;
    }
    public void initialize(){
        m_grabberServo = m_hardwareMap.get(ExpansionHubServo.class,"Grabber Servo");
        m_state = State.RELEASE;
    }
    public void update(){
        if (m_gamepad.right_bumper){
            grab();
        }
        else if (m_gamepad.left_bumper){
            release();
        }
    }
    public void grab(){
        if (m_state == State.RELEASE){
            m_state = State.GRAB;
            m_grabberServo.setPosition(GRAB_POSITION);
        }
    }
    public void release(){
        if (m_state == State.GRAB){
            m_state = State.RELEASE;
            m_grabberServo.setPosition(RELEASE_POSITION);
        }
    }
}
