package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.openftc.revextensions2.ExpansionHubMotor;
@TeleOp(group="test")
public class WinchTest extends LinearOpMode {
    public void runOpMode() throws InterruptedException{
        ExpansionHubMotor winch = hardwareMap.get(ExpansionHubMotor.class,"winch");
        winch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        winch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while(!isStopRequested()){
            if (gamepad1.a){
                winch.setPower(-.3);
            }
            else if (gamepad1.b){
                winch.setPower(.3);
            }
            else{
                winch.setPower(0);
            }
            telemetry.addData("winch power",winch.getPower());
            telemetry.update();
        }
    }
}
