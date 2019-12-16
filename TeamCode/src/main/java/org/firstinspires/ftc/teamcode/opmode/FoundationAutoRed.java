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
import org.firstinspires.ftc.teamcode.subsystems.Foundation;
import org.firstinspires.ftc.teamcode.subsystems.Lift;

import java.util.List;

@Autonomous
public class FoundationAutoRed extends LinearOpMode {

    public void runOpMode() throws InterruptedException{
        MecanumDriveBaseOptimized drive = new MecanumDriveBaseOptimized(hardwareMap,telemetry);
        Foundation foundation = new Foundation(hardwareMap,gamepad1);
        Lift lift = new Lift(hardwareMap,gamepad1);
        drive.initialize();
        foundation.initialize();
        lift.initialize();
        waitForStart();
        foundation.release();
        lift.goUp();
        Thread.sleep(1000);
        lift.stopWinch();

        drive.runToPoseSync(new Pose2d(37,-12,Math.toRadians(-10)));
        foundation.grab();
        Thread.sleep(1000);
        drive.runToPoseSync(new Pose2d(0,-12,Math.toRadians(0)));
        foundation.release();
        Thread.sleep(1000);
        drive.runToPoseSync(new Pose2d(4,40,Math.toRadians(0)));
        Thread.sleep(1000);
    }
}
