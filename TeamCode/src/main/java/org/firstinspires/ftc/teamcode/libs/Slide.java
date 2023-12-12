package org.firstinspires.ftc.teamcode.libs;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Slide {
    public DcMotor slide1;
    public DcMotor slide2;
    public Slide(HardwareMap hardwareMap){
        DcMotor slide1 = hardwareMap.dcMotor.get("slide1");
        DcMotor slide2 = hardwareMap.dcMotor.get("slide2");

    };
    final double SLIDE_POWER = 0.6;
    final double position1 = 0.0;
    final double position2 = 0.0;
    final double position3 = 0.0;
    final double position4 = 0.0;
    public void up(HardwareMap hardwareMap, String position){
        slide1.setPower(SLIDE_POWER);
        slide2.setPower(SLIDE_POWER);
    }
    public void down(HardwareMap hardwareMap, String position){
        slide1.setPower(-SLIDE_POWER);
        slide2.setPower(-SLIDE_POWER);
    }

    public void actuate(double power){
        slide1.setPower(power);
        slide2.setPower(power);
    }

}
