package org.firstinspires.ftc.teamcode.libs;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm {
    Servo arm1;
    Servo arm2;
    private double angle = 0.0;
    final double RELEASE_ANGLE = 172.0;
    final static double RELEASE_POSITION = 1.0;
    final static double INTAKE_POSITION = 0.0;
    final static double READY_POSITION = 0.21;
    private double armOneMinPosition = 0.0;
    private double armOneMaxPosition = 0.48;

    private double armTwoMinPosition = 0.0;
    private double armTwoMaxPosition = 0.48;
    public Arm(HardwareMap hardwareMap, double minPositionArmOne, double maxPositionArmOne, double minPositionArmTwo, double maxPositionArmTwo){
        arm1 = hardwareMap.servo.get("arm1");
        arm2 = hardwareMap.servo.get("arm2");
        armOneMinPosition = minPositionArmOne;
        armOneMaxPosition = maxPositionArmOne;
        armTwoMaxPosition = maxPositionArmTwo;
        armTwoMinPosition = minPositionArmTwo;
        arm1.scaleRange(minPositionArmOne, maxPositionArmOne);
        arm2.scaleRange(minPositionArmTwo, maxPositionArmTwo);

        angle = RELEASE_ANGLE;
    }

    public void turn(double position){
        // ONLY TURN WHEN THE INTAKE IS NOT AT THE RECEIVING POSITION
        arm1.setPosition(position);


    }

}
