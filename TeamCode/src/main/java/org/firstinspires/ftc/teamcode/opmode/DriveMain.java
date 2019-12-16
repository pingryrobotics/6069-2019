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
import org.firstinspires.ftc.teamcode.subsystems.Foundation;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Subsystem;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.ExpansionHubServo;
import org.openftc.revextensions2.RevBulkData;

import java.util.ArrayList;
import java.util.List;

@TeleOp(group="drive",name = "1 Controller Drive")
public class DriveMain extends LinearOpMode {
    private List<Subsystem> subsystems = new ArrayList<>(3);
    @Override
    public void runOpMode() throws InterruptedException{
        MecanumDriveBaseOptimized drive = new MecanumDriveBaseOptimized(hardwareMap,telemetry);
        Lift lift = new Lift(hardwareMap,gamepad1);
        Foundation foundation = new Foundation(hardwareMap,gamepad1);
        subsystems.add(foundation);
        subsystems.add(lift);
        subsystems.add(drive);
        for(Subsystem subsystem : subsystems){
            subsystem.initialize();
        }
        waitForStart();
        while(!isStopRequested()) {
            for(Subsystem subsystem : subsystems){
                subsystem.update();
            }
            Vector2d translationRotatedForFieldCentric = new Vector2d(-gamepad1.left_stick_x, -gamepad1.left_stick_y);
            drive.setDrivePower(new Pose2d(translationRotatedForFieldCentric.getY(),translationRotatedForFieldCentric.getX(), -gamepad1.right_stick_x));
            telemetry.addData("Current Pose:",drive.getCurrentPose());
            telemetry.addData("current velocity",drive.getWheelVelocities());
            telemetry.addData("dt",drive.dt);
            telemetry.update();
        }
    }
}
