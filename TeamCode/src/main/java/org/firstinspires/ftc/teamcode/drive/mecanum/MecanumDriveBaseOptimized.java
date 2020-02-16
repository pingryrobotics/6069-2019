package org.firstinspires.ftc.teamcode.drive.mecanum;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.kinematics.MecanumKinematics;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.Subsystem;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.firstinspires.ftc.teamcode.util.*;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.BASE_CONSTRAINTS;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.WHEEL_BASE;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.encoderTicksToInches;

public class MecanumDriveBaseOptimized extends MecanumDrive implements Subsystem {
    public enum Mode {
        IDLE,
        FOLLOW_TRAJECTORY,
        RUN_TO_POSE
    }
    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(.2,0.001,.01);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(.2,0.001,0.01);
    public PIDFController xTranslationController = new PIDFController(TRANSLATIONAL_PID);
    public PIDFController yTranslationController = new PIDFController(TRANSLATIONAL_PID);
    public PIDFController headingController = new PIDFController(HEADING_PID);
    private DriveConstraints constraints;
    private TrajectoryFollower follower;
    private ExpansionHubEx hub;
    private ElapsedTime elapsedTime;
    private ExpansionHubMotor leftFront, leftRear, rightRear, rightFront;
    private List<ExpansionHubMotor> motors;
    public BNO055IMU imu;
    public double dt;
    private double prevTime;
    private Pose2d m_currentDesiredPose;
    private double currTime;
    private Telemetry m_telemetry;
    private Pose2d currentPose = new Pose2d(0,0,0);
    public Mode mode;
    private Pose2d robotVelocity = new Pose2d(0,0,0);
    private HardwareMap m_hardwareMap;
    public MecanumDriveBaseOptimized(HardwareMap hardwareMap, Telemetry telemetry) {
        super(0,0,0,TRACK_WIDTH,WHEEL_BASE);
        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);
        m_hardwareMap = hardwareMap;
        m_telemetry = telemetry;
    }
    public void initialize(){
        hub = m_hardwareMap.get(ExpansionHubEx.class, "controlHub");

        imu = m_hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        leftFront = m_hardwareMap.get(ExpansionHubMotor.class, "leftFront");
        leftRear = m_hardwareMap.get(ExpansionHubMotor.class, "leftRear");
        rightRear = m_hardwareMap.get(ExpansionHubMotor.class, "rightRear");
        rightFront = m_hardwareMap.get(ExpansionHubMotor.class, "rightFront");

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        for (ExpansionHubMotor motor : motors) {
            if (RUN_USING_ENCODER) {
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // TODO: reverse any motors using DcMotor.setDirection()
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        constraints = new MecanumConstraints(BASE_CONSTRAINTS, TRACK_WIDTH);
        follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID);
        elapsedTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        prevTime = elapsedTime.time()/(double)1000;
    }
    public void update(){
        updateOdometry();
        m_telemetry.addData("in range",isInRange());
        if (mode == Mode.RUN_TO_POSE){
            if (isInRange()){
                setDrivePower(new Pose2d(0,0,0));
                mode = Mode.IDLE;
            }
            else {
                setDrivePower(new Pose2d(xTranslationController.update(currentPose.getX(), robotVelocity.getX()),
                        yTranslationController.update(currentPose.getY(), robotVelocity.getY()),
                        headingController.update(currentPose.getHeading(), robotVelocity.getHeading())));
            }
        }
        m_telemetry.addData("Desired Pose",m_currentDesiredPose);
        m_telemetry.addData("Pose Error",new Pose2d(xTranslationController.getLastError(),yTranslationController.getLastError(),headingController.getLastError()));
        m_telemetry.update();
    }
    public void runToPose(Pose2d desiredPose){
        xTranslationController.reset();
        yTranslationController.reset();
        headingController.reset();
        xTranslationController.setTargetPosition(desiredPose.getX());
        yTranslationController.setTargetPosition(desiredPose.getY());
        headingController.setTargetPosition(desiredPose.getHeading());
        xTranslationController.update(currentPose.getX(), robotVelocity.getX());
        yTranslationController.update(currentPose.getY(), robotVelocity.getY());
        headingController.update(currentPose.getHeading(), robotVelocity.getHeading());
    }
    public void runToPoseSync(Pose2d desiredPose){
        mode = Mode.RUN_TO_POSE;
        m_currentDesiredPose = desiredPose;
        runToPose(desiredPose);
        waitForIdle();
    }
    public void waitForIdle(){
        while(!Thread.currentThread().isInterrupted()&&isBusy()){
            update();
        }
    }
    public boolean isInRange(){
        boolean inRange = true;
        if(Math.abs(headingController.getLastError()) > .15){
            inRange = false;
        }
        if(Math.abs(xTranslationController.getLastError()) > .3) {
            inRange = false;
        }
        if(Math.abs(yTranslationController.getLastError()) > .3){
            inRange = false;
        }
        return inRange;
    }
    public void updateOdometry(){
        List<Double> wheelVelocities = this.getWheelVelocities();
        currTime = elapsedTime.time()/(double)1000;
        dt = currTime - prevTime;
        prevTime = currTime;
        Pose2d robotRelativeVelocity = MecanumKinematics.wheelToRobotVelocities(wheelVelocities, TRACK_WIDTH, WHEEL_BASE);
        robotVelocity = robotRelativeVelocity;
        Pose2d fieldRelativeVelocity = new Pose2d(new Vector2d(robotRelativeVelocity.getX(),robotRelativeVelocity.getY()).rotated(this.getRawExternalHeading()),robotRelativeVelocity.getHeading());
        Pose2d poseDelta = fieldRelativeVelocity.times(dt);
        currentPose = currentPose.plus(poseDelta);
    }
    public Pose2d getCurrentPose(){
        return currentPose;
    }
    public List<Double> getWheelPositions() {
        RevBulkData bulkData = hub.getBulkInputData();

        if (bulkData == null) {
            return Arrays.asList(0.0, 0.0, 0.0, 0.0);
        }

        List<Double> wheelPositions = new ArrayList<>();
        for (ExpansionHubMotor motor : motors) {
            wheelPositions.add(encoderTicksToInches(bulkData.getMotorCurrentPosition(motor)));
        }
        return wheelPositions;
    }
    public List<Double> getWheelVelocities() {
        RevBulkData bulkData = hub.getBulkInputData();

        if (bulkData == null) {
            return Arrays.asList(0.0, 0.0, 0.0, 0.0);
        }

        List<Double> wheelVelocities = new ArrayList<>();
        for (ExpansionHubMotor motor : motors) {
            wheelVelocities.add(encoderTicksToInches(bulkData.getMotorVelocity(motor)));
            //wheelVelocities.add((double)bulkData.getMotorVelocity(motor));
        }
        return wheelVelocities;
    }
    public Pose2d getRobotVelocity(){
        return robotVelocity;
    }
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        leftFront.setPower(v);
        leftRear.setPower(v1);
        rightRear.setPower(v2);
        rightFront.setPower(v3);
    }
    public boolean isBusy() {
        return mode != Mode.IDLE;
    }
    public double getRawExternalHeading() {
        return imu.getAngularOrientation().firstAngle;
    }
}
