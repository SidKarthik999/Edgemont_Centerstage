package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class MecanumTeleOp extends LinearOpMode {

    final double LINEAR_SLIDES_MOTOR_PPR = 384.5;
    final double INCHES_PER_SPOOL_REVOLUTION = 3.9;
    final double slidePower = 0.6;
    final int SAFE_POSITION = 2 * INCHES_PER_SPOOL_REVOLUTION * LINEAR_SLIDES_MOTOR_PPR;
    final int TARGET_POSITION_REST = 0.0;

    final int TARGET_POSITION1 = SAFE_POSITION;
    final int TARGET_POSITION2 = 0.0;



    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("LeftFront");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("LeftBack");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("RightFront");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("RightBack");
        CRServo intake = hardwareMap.crservo.get("intake");
        Servo arm1 = hardwareMap.servo.get("arm1");
        Servo arm2 = hardwareMap.servo.get("arm2");
        DcMotor slide1 = hardwareMap.dcMotor.get("slide1");
        DcMotor slide2 = hardwareMap.dcMotor.get("slide2");

        // ORDER OF ACTUATION (MOVING THE ROBOT)
        // 1. Intake (spin the entrapption stars shaft)
        // 2. Move linear slide up to a certain distance so that when you turn the arm
        //      it doesn't get caught in the intake (entrapption stars)
        // 3. Rotate the left and right arms to deliver the pixel to the back of the robot
        //      where the backdrop will be situated

//        Slide slide = new Slide(hardwareMap);
//        Intake intakeClass = new Intake(hardwareMap);
        boolean yWasPressed = false;
        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.

        // DRIVE TRAIN INITIALIZATION
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        // LINEAR SLIDES INITIALIZATION
        slide1.setDirection(DcMotorSimple.Direction.FORWARD);
        slide2.setDirection(DcMotorSimple.Direction.REVERSE);

        slide1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        slide1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        // INTAKE INITIALIZATION
        intake.setDirection(DcMotorSimple.Direction.FORWARD);

        // ARMS INITIALIZATION
        arm1.setDirection(Servo.Direction.FORWARD);
        arm2.setDirection(Servo.Direction.REVERSE);


        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;
            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            String spinning;

            if(gamepad1.left_bumper) { // Move the Intake (Entrapption Stars)
               intake.setPower(0.6);

               spinning = "in";
               telemetry.addData("IsServoSpinning", spinning);
               telemetry.update();
            }
            else if(gamepad1.right_bumper) {
                intake.setPower(-0.6);

                spinning = "out";
                telemetry.addData("IsServoSpinning", spinning);
                telemetry.update();
            }else{
                intake.setPower(0.0);
                spinning = "no";
                telemetry.addData("IsServoSpinning", spinning);
                telemetry.update();
            }

            if(gamepad1.dpad_up) {  // Move the slides in tandem
                slide1.setTargetPosition(TARGET_POSITION1);
                slide2.setPower(TARGET_POSITION1);
            }
            else if(gamepad1.dpad_down) {
                slide1.setPower(TARGET_POSITION2);
                slide2.setPower(TARGET_POSITION2);
            }else {
                slide1.setPower(TARGET_POSITION_REST);
                slide2.setPower(TARGET_POSITION_REST);
            }

            if(gamepad1.a) { // Move the Arm Servos

                if (slide1.getCurrentPosition() >= SAFE_POSITION){
                    arm1.setPosition(0.0);
                    arm2.setPosition(0.0);
                }

            }
            if(gamepad1.b) {
                arm1.setPosition(0.5);
                arm2.setPosition(0.5);
            }
            if(gamepad1.y) {
                arm1.setPosition(1.0);
                arm2.setPosition(1.0);
            }

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            if(gamepad1.y){
                yWasPressed = !yWasPressed;
                telemetry.addData("Slow Mode On: ", yWasPressed);
                telemetry.update();
            }
            double frontLeftPower;
            double frontRightPower;
            double backLeftPower;
            double backRightPower;

            // MOVE DRIVE TRAIN
            if(yWasPressed) {
                frontLeftPower = 0.5*((y + x + rx) / denominator);
                backLeftPower = 0.5*((y - x + rx) / denominator);
                frontRightPower = 0.5*((y - x - rx) / denominator);
                backRightPower = 0.5*((y + x - rx) / denominator);
            }else{
                frontLeftPower = (y + x + rx) / denominator;
                backLeftPower = (y - x + rx) / denominator;
                frontRightPower = (y - x - rx) / denominator;
                backRightPower = (y + x - rx) / denominator;
            }

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
            telemetry.addData("Position of Slide One: ", slide1.getCurrentPosition());
            telemetry.addData("Position of Slide Two: ", slide2.getCurrentPosition());
            telemetry.update();
        }
    }


}