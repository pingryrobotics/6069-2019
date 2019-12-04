package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.kinematics.MecanumKinematics;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.GEAR_RATIO;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.WHEEL_RADIUS;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.encoderTicksToInches;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.mecanum.MecanumDriveBaseOptimized;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.ExpansionHubServo;
import org.openftc.revextensions2.RevBulkData;
import java.util.List;

@TeleOp(group="drive")
public class DriveMain extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException{
        MecanumDriveBaseOptimized drive = new MecanumDriveBaseOptimized(hardwareMap);
        waitForStart();
        double dt;
        double last;
        double now;
        last = System.currentTimeMillis();
        Pose2d currentPose = new Pose2d(0,0,drive.getRawExternalHeading());

        double posIn = .1;
        double posOut = .9;
        while(!isStopRequested()) {
            Vector2d translationRotatedForFieldCentric = new Vector2d(-gamepad1.left_stick_x, -gamepad1.left_stick_y).rotated(drive.getRawExternalHeading());
            drive.setDrivePower(new Pose2d(translationRotatedForFieldCentric.getY(),translationRotatedForFieldCentric.getX(), -gamepad1.right_stick_x/2));
            List<Double> wheelVelocities = drive.getWheelVelocities();
            now = System.currentTimeMillis();
            dt = now - last;
            dt = dt / ((double) 1000);
            Pose2d poseDelta = MecanumKinematics.wheelToRobotVelocities(wheelVelocities, DriveConstants.TRACK_WIDTH, DriveConstants.WHEEL_BASE).times(dt);
            poseDelta = new Pose2d(new Vector2d(poseDelta.getX(),poseDelta.getY()).rotated(drive.getRawExternalHeading()),poseDelta.getHeading());
            currentPose = currentPose.plus(poseDelta);
            last = now;
            telemetry.addData("Wheel Velocities",wheelVelocities);
            telemetry.addData("Pose Delta:",poseDelta);
            telemetry.addData("Current Pose:",currentPose);
            telemetry.update();
        }
    }
}
