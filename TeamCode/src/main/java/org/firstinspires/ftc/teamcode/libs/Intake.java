package org.firstinspires.ftc.teamcode.libs;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {

    static final double spinInPower = 0.6;
    static final double spinOutPower = -0.6;
    public CRServo intake;
    public Intake(HardwareMap hardwareMap){
        CRServo intake = hardwareMap.crservo.get("intake");
    }
    public void spinIn(HardwareMap hardwareMap){

        intake.setPower(spinInPower);
    }
    public void spinOut(HardwareMap hardwareMap){

        intake.setPower(spinOutPower);
    }
    public void stop(HardwareMap hardwareMap) {
        intake.setPower(0.);
    }
}
