package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "drive")
public class CompetitionAutoBlue extends LinearOpMode {
    public static double DISTANCE = 60; // in
    CRServo intake = hardwareMap.get(CRServo.class, "intake");
    Servo arm1 = hardwareMap.get(Servo.class, "arm1");
    Servo arm2 = hardwareMap.get(Servo.class, "arm2");
    DcMotorImplEx slide1 = hardwareMap.get(DcMotorImplEx.class,"slide1");
    DcMotorImplEx slide2 = hardwareMap.get(DcMotorImplEx.class,"slide2");


    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory trajectory = drive.trajectoryBuilder(new Pose2d())
                //scan
                .forward(40)
                .build();
        Trajectory trajectory2 = drive.trajectoryBuilder(new Pose2d()).back(30).build();

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(trajectory);
        drive.turn(45);

        drive.turn(-45);
        drive.followTrajectory(trajectory2);
        drive.turn(90);

        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();

        while (!isStopRequested() && opModeIsActive()) ;
    }
}
