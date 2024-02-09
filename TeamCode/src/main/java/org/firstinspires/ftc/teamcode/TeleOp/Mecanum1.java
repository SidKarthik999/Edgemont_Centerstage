package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp
public class Mecanum1 extends LinearOpMode {

    final double LINEAR_SLIDES_MOTOR_PPR = 384.5;
    final double INCHES_PER_SPOOL_REVOLUTION = 3.77;
    final double slidePower = 0.6;

    final int TARGET_POSITION_REST = 0;
    final int SLIDE_TICKS_PER_INCH = (int) Math.round(LINEAR_SLIDES_MOTOR_PPR/INCHES_PER_SPOOL_REVOLUTION);
    final int SAFE_POSITION = (int)(SLIDE_TICKS_PER_INCH*8.5);
    final int FIRST_BACKDROP_MARK = (int)(SLIDE_TICKS_PER_INCH*5);
    final int SECOND_BACKDROP_MARK = (int)(SLIDE_TICKS_PER_INCH*12);
    final int THIRD_BACKDROP_MARK = (int)(SLIDE_TICKS_PER_INCH*18);
    final double SAFE_SLIDE_VELOCITY = (double)(480.625);
    final int TARGET_POSITION1 = SAFE_POSITION;
    final int TARGET_POSITION2 = 385*2;
    final double DELIVERED_POSITION = 0.6;
    final double INTAKE_POSITION = 0.055;
    final double ARM_SAFE_POSITION = 0.25;

    final int OPTIMAL_DROP_POSITION = (int) (SLIDE_TICKS_PER_INCH*3.0);

    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
//        DcMotorImplEx frontLeftMotor = hardwareMap.get(DcMotorImplEx.class, "LeftFront");
//        DcMotorImplEx backLeftMotor = hardwareMap.get(DcMotorImplEx.class,"LeftBack");
//        DcMotorImplEx frontRightMotor = hardwareMap.get(DcMotorImplEx.class, "RightFront");
//        DcMotorImplEx backRightMotor = hardwareMap.get(DcMotorImplEx.class,"RightBack");
//        Servo intake = hardwareMap.get(Servo.class, "intake");
        CRServo intake = hardwareMap.get(CRServo.class, "intake");
//        Servo arm1 = hardwareMap.get(Servo.class, "arm1");
//        Servo arm2 = hardwareMap.get(Servo.class, "arm2");
        DcMotorImplEx slide1 = hardwareMap.get(DcMotorImplEx.class,"slide1");
        DcMotorImplEx slide2 = hardwareMap.get(DcMotorImplEx.class,"slide2");

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
//        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
//        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
//        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        // LINEAR SLIDES INITIALIZATION
        slide1.setDirection(DcMotorSimple.Direction.REVERSE);
        slide2.setDirection(DcMotorSimple.Direction.FORWARD);
        slide1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //slide1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //slide2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        slide1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // INTAKE INITIALIZATION
//        intake.setDirection(Servo.Direction.FORWARD);
//        intake.setDirection(CRServo.Direction.FORWARD);

        // ARMS INITIALIZATION
//        arm1.setDirection(Servo.Direction.FORWARD);
//        arm2.setDirection(Servo.Direction.REVERSE);
//        arm1.setPosition(INTAKE_POSITION);
//        arm2.setPosition(INTAKE_POSITION);



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
//            telemetry.addData("Position of Arm 1: ", arm1.getPosition());
//            telemetry.addData("Position of Arm 2: ", arm2.getPosition());
            telemetry.update();

            if(gamepad1.left_bumper) { // Move the Intake (Entrapption Stars)
                intake.setPower(-1.0);

                spinning = "in";
//               telemetry.addData("IsServoSpinning", spinning);
//               telemetry.update();
            }
            else if(gamepad1.right_bumper) {
                intake.setPower(1.0);

                spinning = "out";
//                telemetry.addData("IsServoSpinning", spinning);
//                telemetry.update();
            }
            else{
                intake.setPower(0.0);
                spinning = "no";
//                telemetry.addData("IsServoSpinning", spinning);
//                telemetry.update();
            }
//
            if(gamepad2.dpad_up) {  // Move the slides in tandem
                slide1.setTargetPosition(THIRD_BACKDROP_MARK);
                slide1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slide2.setTargetPosition(THIRD_BACKDROP_MARK);
                slide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slide1.setVelocity(SAFE_SLIDE_VELOCITY);
                slide2.setVelocity(SAFE_SLIDE_VELOCITY);
                telemetry.addData("Position of Slide One: ", slide1.getCurrentPosition());
                telemetry.addData("Current of Slide 1: ", slide1.getCurrent(CurrentUnit.MILLIAMPS));
                telemetry.addData("Position of Slide Two: ", slide2.getCurrentPosition());
                telemetry.addData("Current of Slide 2: ", slide2.getCurrent(CurrentUnit.MILLIAMPS));
//                telemetry.addData("Position of Arm 1: ", arm1.getPosition());
//                telemetry.addData("Position of Arm 2: ", arm2.getPosition());
            }
            else if(gamepad2.dpad_down) {

                slide1.setTargetPosition(0);
                slide1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slide2.setTargetPosition(0);
                slide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slide1.setVelocity(SAFE_SLIDE_VELOCITY);
                slide2.setVelocity(SAFE_SLIDE_VELOCITY);
                telemetry.addData("Position of Slide One: ", slide1.getCurrentPosition());
                telemetry.addData("Current of Slide 1: ", slide1.getCurrent(CurrentUnit.MILLIAMPS));
                telemetry.addData("Position of Slide Two: ", slide2.getCurrentPosition());
                telemetry.addData("Current of Slide 2: ", slide2.getCurrent(CurrentUnit.MILLIAMPS));
//                telemetry.addData("Position of Arm 1: ", arm1.getPosition());
//                telemetry.addData("Position of Arm 2: ", arm2.getPosition());
            }
            else if(gamepad2.dpad_right) {

                slide1.setTargetPosition(SAFE_POSITION);
                slide1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slide2.setTargetPosition(SAFE_POSITION);
                slide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slide1.setVelocity(SAFE_SLIDE_VELOCITY);
                slide2.setVelocity(SAFE_SLIDE_VELOCITY);
                telemetry.addData("Position of Slide One: ", slide1.getCurrentPosition());
                telemetry.addData("Current of Slide 1: ", slide1.getCurrent(CurrentUnit.MILLIAMPS));
                telemetry.addData("Position of Slide Two: ", slide2.getCurrentPosition());
                telemetry.addData("Current of Slide 2: ", slide2.getCurrent(CurrentUnit.MILLIAMPS));
//                telemetry.addData("Position of Arm 1: ", arm1.getPosition());
//                telemetry.addData("Position of Arm 2: ", arm2.getPosition());
            }
            else if(gamepad2.dpad_left) {

                slide1.setTargetPosition(SECOND_BACKDROP_MARK);
                slide1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slide2.setTargetPosition(SECOND_BACKDROP_MARK);
                slide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slide1.setVelocity(SAFE_SLIDE_VELOCITY);
                slide2.setVelocity(SAFE_SLIDE_VELOCITY);
                telemetry.addData("Position of Slide One: ", slide1.getCurrentPosition());
                telemetry.addData("Current of Slide 1: ", slide1.getCurrent(CurrentUnit.MILLIAMPS));
                telemetry.addData("Position of Slide Two: ", slide2.getCurrentPosition());
                telemetry.addData("Current of Slide 2: ", slide2.getCurrent(CurrentUnit.MILLIAMPS));
//                telemetry.addData("Position of Arm 1: ", arm1.getPosition());
//                telemetry.addData("Position of Arm 2: ", arm2.getPosition());
            }
//
            else if(gamepad2.left_stick_button) {
                slide1.setTargetPosition(100*2);
                slide2.setTargetPosition(100*2);
                slide1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slide1.setVelocity(SAFE_SLIDE_VELOCITY);
                slide2.setVelocity(SAFE_SLIDE_VELOCITY);
                telemetry.addData("Position of Slide One: ", slide1.getCurrentPosition());
                telemetry.addData("Current of Slide 1: ", slide1.getCurrent(CurrentUnit.MILLIAMPS));
                telemetry.addData("Position of Slide Two: ", slide2.getCurrentPosition());
                telemetry.addData("Current of Slide 2: ", slide2.getCurrent(CurrentUnit.MILLIAMPS));
//                telemetry.addData("Position of Arm 1: ", arm1.getPosition());
//                telemetry.addData("Position of Arm 2: ", arm2.getPosition());
            }
//            }else {
////                slide1.setTargetPosition(TARGET_POSITION_REST);
////                slide2.setTargetPosition(TARGET_POSITION_REST);
////                sleep(500);
////                slide1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////                slide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                slide1.setVelocity(0.0);
//                slide2.setVelocity(0.0);
//            }
//
//            if(gamepad2.a) { // Move the Arm Servos
//
//                arm1.setPosition(INTAKE_POSITION);
//                arm2.setPosition(INTAKE_POSITION);
//
//            }
//            if(gamepad2.b) {
//                if (slide1.getCurrentPosition() >= OPTIMAL_DROP_POSITION){
//                    arm1.setPosition(DELIVERED_POSITION);
//                    arm2.setPosition(DELIVERED_POSITION);
//                }
//            }
//            if(gamepad2.y) {
////                if (slide1.getCurrentPosition() >= SAFE_POSITION){
//                arm1.setPosition(0.08);
//                arm2.setPosition(0.08);
////                }
//
//            }
//            if(gamepad2.x) {
//                if (slide1.getCurrentPosition() >= SAFE_POSITION){
//                    arm1.setPosition(ARM_SAFE_POSITION);
//                    arm2.setPosition(ARM_SAFE_POSITION);
//                }
//            }
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

//            frontLeftMotor.setPower(frontLeftPower);
//            backLeftMotor.setPower(backLeftPower);
//            frontRightMotor.setPower(frontRightPower);
//            backRightMotor.setPower(backRightPower);

//            telemetry.addData("Position of Slide One: ", slide1.getCurrentPosition());
//            telemetry.addData("Position of Slide Two: ", slide2.getCurrentPosition());
//            telemetry.addData("Power of Slide One: ", slide1.getVelocity());
//            telemetry.addData("Power of Slide Two: ", slide2.getVelocity());
//            telemetry.addData("Position of Arm 1: ", arm1.getPosition());
//            telemetry.addData("Position of Arm 2: ", arm2.getPosition());
            telemetry.update();
        }
    }


}
