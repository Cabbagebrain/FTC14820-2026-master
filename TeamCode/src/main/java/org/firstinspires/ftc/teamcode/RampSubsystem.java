package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class RampSubsystem {
    private Servo rightServo;
    private Servo leftServo;

    //TODO figure out the pos servo needs to turn to lift ramp
    private final double liftpos = 0;

    //Note
    public RampSubsystem(HardwareMap hardwareMap) {
        rightServo = hardwareMap.get(Servo.class, "rightramp"); //port 0
        leftServo = hardwareMap.get(Servo.class, "leftramp"); //port 1

        //TODO unsure if these are the right directions
        rightServo.setDirection(Servo.Direction.FORWARD);
        leftServo.setDirection(Servo.Direction.REVERSE);
    }

    public void liftRamp() {
        rightServo.setPosition(liftpos);
        leftServo.setPosition(liftpos);
    }

    public void dropRamp() {
        rightServo.setPosition(0);
        leftServo.setPosition(0);
    }
}
