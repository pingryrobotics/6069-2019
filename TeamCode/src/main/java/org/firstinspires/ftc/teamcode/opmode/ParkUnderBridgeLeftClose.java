package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.mecanum.MecanumDriveBaseOptimized;
@Autonomous
public class ParkUnderBridgeLeftClose extends LinearOpMode {
    public void runOpMode(){
        MecanumDriveBaseOptimized drive = new MecanumDriveBaseOptimized(hardwareMap,telemetry);
        drive.initialize();
        waitForStart();
        drive.runToPoseSync(new Pose2d(4,12,0));
    }
}
