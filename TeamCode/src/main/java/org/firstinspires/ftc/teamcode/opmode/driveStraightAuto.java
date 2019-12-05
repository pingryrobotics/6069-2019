package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.kinematics.MecanumKinematics;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.control.PIDCoefficients;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.mecanum.MecanumDriveBaseOptimized;

import java.util.List;
@Autonomous
public class driveStraightAuto extends LinearOpMode {
    double dt;
    double last;
    double now;
    Pose2d currentPose = new Pose2d(0,0,0);
    PIDFController xTranslationController = new PIDFController(new PIDCoefficients(.05,0,.0005));
    PIDFController yTranslationController = new PIDFController(new PIDCoefficients(.05,0,.0005));
    PIDFController headingController = new PIDFController(new PIDCoefficients(.05,0.0001,0.002));
    public void runOpMode(){
        MecanumDriveBaseOptimized drive = new MecanumDriveBaseOptimized(hardwareMap);
        waitForStart();
        last = System.currentTimeMillis();
        xTranslationController.setTargetPosition(36);
        yTranslationController.setTargetPosition(0);
        headingController.setTargetPosition(Math.toRadians(30));
        while(!isStopRequested()){
            List<Double> wheelVelocities = drive.getWheelVelocities();
            now = System.currentTimeMillis();
            dt = now - last;
            dt = dt / ((double) 1000);
            Pose2d robotFrameVelocity = MecanumKinematics.wheelToRobotVelocities(wheelVelocities, DriveConstants.TRACK_WIDTH, DriveConstants.WHEEL_BASE);
            Pose2d robotFrameVelocityRotated = new Pose2d(new Vector2d(robotFrameVelocity.getX(),robotFrameVelocity.getY()).rotated(drive.getRawExternalHeading()),robotFrameVelocity.getHeading());
            Pose2d poseDelta = robotFrameVelocityRotated.times(dt);
            currentPose = currentPose.plus(poseDelta);
            last = now;
            drive.setDrivePower(new Pose2d(xTranslationController.update(currentPose.getX(),robotFrameVelocity.getX()),
            yTranslationController.update(currentPose.getY(),robotFrameVelocity.getY()),
            headingController.update(currentPose.getHeading(),robotFrameVelocity.getHeading())));
            telemetry.addData("Current Pose:",currentPose);
            telemetry.update();
        }
    }
}
