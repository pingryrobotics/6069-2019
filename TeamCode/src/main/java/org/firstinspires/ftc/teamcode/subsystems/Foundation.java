package org.firstinspires.ftc.teamcode.subsystems;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.openftc.revextensions2.ExpansionHubServo;

public class Foundation {
    private ExpansionHubServo leftServo;
    private ExpansionHubServo rightServo;
    //TODO: test servo bounds for actual position values
    private final double kLeftGrabPosition = .5;
    private final double kLeftReleasePosition = .9;
    private final double kRightGrabPosition = .5;
    private final double kRightReleasePosition = .9;
    private State state;
    public enum State{
        GRAB,
        RELEASE
    }
    public Foundation(HardwareMap hardwareMap){
        leftServo = hardwareMap.get(ExpansionHubServo.class,"leftServo");
        rightServo = hardwareMap.get(ExpansionHubServo.class,"rightServo");
    }
    public void grab(){
        leftServo.setPosition(kLeftGrabPosition);
        rightServo.setPosition(kRightGrabPosition);
        this.state = State.GRAB;
    }
    public void release(){
        leftServo.setPosition(kLeftReleasePosition);
        rightServo.setPosition(kRightReleasePosition);
        this.state = State.RELEASE;
    }
}
