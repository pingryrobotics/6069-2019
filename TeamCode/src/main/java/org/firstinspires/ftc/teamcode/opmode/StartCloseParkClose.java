package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.kinematics.MecanumKinematics;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.mecanum.MecanumDriveBaseOptimized;

import java.util.List;

@Autonomous
public class StartCloseParkClose extends LinearOpMode {
    double dt;
    double last;
    double now;
    Pose2d currentPose = new Pose2d(0,0,0);

    public void runOpMode() throws InterruptedException{
        MecanumDriveBaseOptimized drive = new MecanumDriveBaseOptimized(hardwareMap);
        drive.initialize();
        waitForStart();
        drive.runToPoseSync(new Pose2d(10,10,Math.toRadians(-45)));
    }
}
