package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.openftc.revextensions2.ExpansionHubMotor;
@TeleOp(group="test")
public class WinchTest extends LinearOpMode {
    private Lift lift;
    public void runOpMode() throws InterruptedException{
        lift = new Lift(hardwareMap,gamepad1);
        lift.initialize();
        waitForStart();
        while(!isStopRequested()){
            lift.update();
            telemetry.update();
        }
    }
}
