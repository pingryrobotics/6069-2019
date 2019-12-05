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
    double dt;
    double last;
    double now;
    Pose2d currentPose = new Pose2d(0,0,0);
    @Override
    public void runOpMode() throws InterruptedException{
        MecanumDriveBaseOptimized drive = new MecanumDriveBaseOptimized(hardwareMap);
        waitForStart();
        double posIn = .1;
        double posOut = .9;
        ExpansionHubMotor winch = hardwareMap.get(ExpansionHubMotor.class,"winch");
        winch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        winch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        last = System.currentTimeMillis();
        while(!isStopRequested()) {
            Vector2d translationRotatedForFieldCentric = new Vector2d(-gamepad1.left_stick_x, -gamepad1.left_stick_y).rotated(drive.getRawExternalHeading());
            drive.setDrivePower(new Pose2d(translationRotatedForFieldCentric.getY(),translationRotatedForFieldCentric.getX(), -gamepad1.right_stick_x/4));
            List<Double> wheelVelocities = drive.getWheelVelocities();
            now = System.currentTimeMillis();
            dt = now - last;
            dt = dt / ((double) 1000);
            Pose2d robotFrameVelocity = MecanumKinematics.wheelToRobotVelocities(wheelVelocities, DriveConstants.TRACK_WIDTH, DriveConstants.WHEEL_BASE);
            Pose2d robotFrameVelocityRotated = new Pose2d(new Vector2d(robotFrameVelocity.getX(),robotFrameVelocity.getY()).rotated(drive.getRawExternalHeading()),robotFrameVelocity.getHeading());
            Pose2d poseDelta = robotFrameVelocityRotated.times(dt);
            currentPose = currentPose.plus(poseDelta);
            last = now;
            if (gamepad1.a){
                winch.setPower(-.5);
            }
            else if (gamepad1.b){
                winch.setPower(.5);
            }
            else{
                winch.setPower(0);
            }
            telemetry.addData("Wheel Velocities",wheelVelocities);
            telemetry.addData("Pose Delta:",poseDelta);
            telemetry.addData("Current Pose:",currentPose);
            telemetry.update();
        }
    }
}
