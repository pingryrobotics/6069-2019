package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Slides{
    private DcMotor left;
    private DcMotor right;
    public Slides(HardwareMap hardwareMap){
        left = hardwareMap.get(DcMotor.class, "left");
        right = hardwareMap.get(DcMotor.class, "right");

        //NOTE: IF IT GOES BACKWARD SWITCH RIGHT TO FORWARD AND LEFT TO REVERSE
        left.setDirection(DcMotor.Direction.FORWARD);
        right.setDirection(DcMotor.Direction.REVERSE);
    }
    public void moveUp(){
        left.setPower(.6);
        right.setPower(.6);
    }
    public void moveDown(){
        left.setPower(-.6);
        right.setPower(-.6);
    }


}