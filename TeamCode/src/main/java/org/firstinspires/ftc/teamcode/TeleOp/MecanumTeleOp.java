package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class MecanumTeleOp extends LinearOpMode {
    final double slidePower = 0.6;

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

//        Slide slide = new Slide(hardwareMap);
//        Intake intakeClass = new Intake(hardwareMap);
        boolean yWasPressed = false;
        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);


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
            if(gamepad1.left_bumper) {
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
                intake.setPower(0.);
                spinning = "no";
                telemetry.addData("IsServoSpinning", spinning);
                telemetry.update();
            }
            if(gamepad1.dpad_up) {
                actuate(slidePower);
            }
            else if(gamepad1.dpad_down) {
                actuate(-slidePower);
            }else {
                actuate(0);
            }
            if(gamepad1.a) {
                arm1.setPosition(0.0);
                arm2.setPosition(1.0);
            }
            if(gamepad1.b) {
                arm1.setPosition(0.5);
                arm2.setPosition(0.5);
            }
            if(gamepad1.y) {
                arm1.setPosition(1.0);
                arm2.setPosition(0.0);
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

    public void actuate(double power){
        DcMotor slide1 = hardwareMap.dcMotor.get("slide1");
        DcMotor slide2 = hardwareMap.dcMotor.get("slide2");
        slide1.setPower(power);
        slide2.setPower(power);
    }
}