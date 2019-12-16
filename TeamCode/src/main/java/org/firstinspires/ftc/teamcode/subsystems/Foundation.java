package org.firstinspires.ftc.teamcode.subsystems;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.openftc.revextensions2.ExpansionHubServo;

public class Foundation implements Subsystem {
    private ExpansionHubServo leftServo;
    private ExpansionHubServo rightServo;
    //TODO: test servo bounds for actual position values
    private final double kLeftGrabPosition = .5;
    private final double kLeftReleasePosition = .1;
    private final double kRightGrabPosition = .5;
    private final double kRightReleasePosition = .9;
    private State state;
    private Gamepad m_gamepad;
    private HardwareMap m_hardwareMap;
    public enum State{
        GRAB,
        RELEASE
    }
    public Foundation(HardwareMap hardwareMap, Gamepad gamepad){
        m_gamepad = gamepad;
        m_hardwareMap = hardwareMap;
    }
    public void initialize(){
        leftServo = m_hardwareMap.get(ExpansionHubServo.class,"leftServo");
        rightServo = m_hardwareMap.get(ExpansionHubServo.class,"rightServo");
        this.state = State.GRAB;
    }
    public void update(){
        if (m_gamepad.x && this.state!=State.GRAB){
            grab();
        }
        else if (m_gamepad.y && this.state!=State.RELEASE){
            release();
        }
    }
    public void grab(){
        if (this.state == State.RELEASE){
            leftServo.setPosition(kLeftGrabPosition);
            rightServo.setPosition(kRightGrabPosition);
            this.state = State.GRAB;
        }
    }
    public void release(){
        if (this.state == State.GRAB){
            leftServo.setPosition(kLeftReleasePosition);
            rightServo.setPosition(kRightReleasePosition);
            this.state = State.RELEASE;
        }
    }
}
