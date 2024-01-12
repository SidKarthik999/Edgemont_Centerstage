package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;


/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "drive")
public class CompetitionAutoBlue extends LinearOpMode {
    public static double DISTANCE = 60; // in
    final double LINEAR_SLIDES_MOTOR_PPR = 384.5;
    final double INCHES_PER_SPOOL_REVOLUTION = 3.77;
    final int SLIDE_TICKS_PER_INCH = (int) Math.round(LINEAR_SLIDES_MOTOR_PPR/INCHES_PER_SPOOL_REVOLUTION);
    final int SAFE_POSITION = (int)(SLIDE_TICKS_PER_INCH*8.5);
    final double SAFE_SLIDE_VELOCITY = (double)(384.5);
    final double CONVERSION_FACTOR_TURN=245/90;
    final double CONVERSION_FACTOR_DRIVE=44/28.5;
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private static final String[] LABELS = {
            "red_left",
            "red_right",
            "red_center",
            "blue_left",
            "blue_right",
            "blue_center"
    };
    final double slidePower = 0.6;

    final int TARGET_POSITION_REST = 0;

    final int FIRST_BACKDROP_MARK = (int)(SLIDE_TICKS_PER_INCH*5);
    final int SECOND_BACKDROP_MARK = (int)(SLIDE_TICKS_PER_INCH*12);
    final int THIRD_BACKDROP_MARK = (int)(SLIDE_TICKS_PER_INCH*21);
    final int TARGET_POSITION1 = SAFE_POSITION;
    final int TARGET_POSITION2 = 385*2;
    final double DELIVERED_POSITION = 0.6;
    final double INTAKE_POSITION = 0.055;
    final double ARM_SAFE_POSITION = 0.25;
    final int OPTIMAL_DROP_POSITION = (int) (SLIDE_TICKS_PER_INCH*3.0);
    private TfodProcessor tfod;
    private VisionPortal visionPortal;
//    CRServo intake = hardwareMap.get(CRServo.class, "intake");
//    Servo arm1 = hardwareMap.get(Servo.class, "arm1");
//    Servo arm2 = hardwareMap.get(Servo.class, "arm2");




    @Override
    public void runOpMode() throws InterruptedException {

        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        String position="";
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        DcMotorImplEx slide1 = hardwareMap.get(DcMotorImplEx.class,"slide1");
        DcMotorImplEx slide2 = hardwareMap.get(DcMotorImplEx.class,"slide2");
        CRServo intake = hardwareMap.get(CRServo.class, "intake");
        Servo arm1 = hardwareMap.get(Servo.class, "arm1");
        Servo arm2 = hardwareMap.get(Servo.class, "arm2");

        slide1.setDirection(DcMotorSimple.Direction.REVERSE);
        slide2.setDirection(DcMotorSimple.Direction.FORWARD);
        slide1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Trajectory driveForward = drive.trajectoryBuilder(new Pose2d())
                //scan
//                .forward(44)
                .forward(28.5*CONVERSION_FACTOR_DRIVE)
                .build();
        Trajectory driveToSpikeMark = drive.trajectoryBuilder(new Pose2d())
                .forward(5*CONVERSION_FACTOR_DRIVE)
                .build();
        Trajectory driveForwardUntilBackdrop = drive.trajectoryBuilder(new Pose2d())
                .forward(73*CONVERSION_FACTOR_DRIVE)
//                .strafeLeft(27*CONVERSION_FACTOR_DRIVE)
                .build();
        Trajectory strafe = drive.trajectoryBuilder(new Pose2d())
                .strafeLeft(27*CONVERSION_FACTOR_DRIVE)
                .build();
        Trajectory backwards = drive.trajectoryBuilder(new Pose2d())
                .back(28.5*CONVERSION_FACTOR_DRIVE)
                .build();
        waitForStart();

        if (isStopRequested()) return;

        initTfod();
        while(position.equals("")) {
            position=telemetryTfod();
        }
        slide1.setTargetPosition(SAFE_POSITION);
        slide2.setTargetPosition(SAFE_POSITION);
        slide1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide1.setVelocity(SAFE_SLIDE_VELOCITY);
        slide2.setVelocity(SAFE_SLIDE_VELOCITY);
        Thread.sleep(1000);
        drive.followTrajectory(driveForward);
        if(position.equals("left")){
            drive.turn(Math.toRadians(90* CONVERSION_FACTOR_TURN));
//            drive.followTrajectory(driveToSpikeMark);
            intake.setPower(1.0);
            Thread.sleep(1000);
            drive.turn(Math.toRadians(-90 * CONVERSION_FACTOR_TURN));
        }else if(position.equals("right")){
            drive.turn(Math.toRadians(-90 * CONVERSION_FACTOR_TURN));
//            drive.followTrajectory(driveToSpikeMark);
            intake.setPower(1.0);
            Thread.sleep(1000);
            drive.turn(Math.toRadians(90* CONVERSION_FACTOR_TURN));
        }else{
            intake.setPower(1.0);
        }
        drive.followTrajectory(backwards);
        drive.turn(-90*CONVERSION_FACTOR_TURN);
        drive.followTrajectory(driveForwardUntilBackdrop);
        drive.followTrajectory(strafe);
        arm1.setPosition(0.25);
        arm2.setPosition(0.25);
        slide1.setTargetPosition(OPTIMAL_DROP_POSITION);
        slide2.setTargetPosition(OPTIMAL_DROP_POSITION);
        slide1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide1.setVelocity(SAFE_SLIDE_VELOCITY);
        slide2.setVelocity(SAFE_SLIDE_VELOCITY);




        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();

        while (!isStopRequested() && opModeIsActive()) ;
    }
    private void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                // Use setModelAssetName() if the TF Model is built in as an asset.
                // Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                .setModelAssetName("model_20240109_174311.tflite")
//                .setModelFileName("model_20240108_155735.tflite")

                .setModelLabels(LABELS)
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                //.setModelAspectRatio(16.0 / 9.0)

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        // builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableCameraMonitoring(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        tfod.setMinResultConfidence(0.55f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

    }   // end method initTfod()

    /**
     * Function to add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */
    private String telemetryTfod() {
        String position="";
        final int LEFT_EXTREMITY = 125;
        final int RIGHT_EXTREMITY = 515;

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());


        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop() + recognition.getBottom()) / 2 ;

            //IF prop.right < 125 ==> on LEFT
            if (x < LEFT_EXTREMITY) {
                position = "left";
                telemetry.addData("Team Prop is on the", position);

            }
            else if (x > RIGHT_EXTREMITY) {
                position = "right";
                telemetry.addData("Team Prop is on the", position);
                // Team Prop is on RIGHT
                // DO SOMETHING
            }
            else {
                position = "center";
                telemetry.addData("Team Prop is on the", position);
                // Team Prop is on CENTER
                // DO SOMETHING
            }



            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }   // end for() loop

        return position;
    }   // end method telemetryTfod()
}
