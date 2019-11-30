package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.mecanum.MecanumDriveBaseOptimized;
import org.openftc.revextensions2.RevBulkData;

@TeleOp(group="drive")
public class DriveMain extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException{
        MecanumDriveBaseOptimized drive = new MecanumDriveBaseOptimized(hardwareMap);
        waitForStart();
        while(!isStopRequested()){
            drive.setDrivePower(new Pose2d(-gamepad1.left_stick_y,
                                           -gamepad1.left_stick_x,
                                           -gamepad1.right_stick_x));

        }
    }
}
