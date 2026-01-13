package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ShintakeSubsystem {
    private DcMotor RBintake;
    private DcMotor flywheelR;
    private DcMotor flywheelL;
    private DcMotor internalIntake;
    public ShintakeSubsystem(HardwareMap hardwareMap) {
        RBintake = hardwareMap.get(DcMotor.class, "outer intake"); //port 0
        flywheelR = hardwareMap.get(DcMotor.class, "right flywheel"); //port 1
        flywheelL = hardwareMap.get(DcMotor.class, "left flywheel"); //port 2
        internalIntake = hardwareMap.get(DcMotor.class, "internal intake"); //port 3

        internalIntake.setDirection(DcMotor.Direction.REVERSE);
        flywheelL.setDirection(DcMotor.Direction.REVERSE);

        RBintake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheelR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheelL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        internalIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void runIntake(double power) {
        RBintake.setPower(power);
        internalIntake.setPower(power);
    }

    public void stopIntake() {
        RBintake.setPower(0);
        internalIntake.setPower(0);
    }

    public void runFlywheel(double power) {
        flywheelL.setPower(power);
        flywheelR.setPower(power);
    }

    public void stopFlywheel() {
        flywheelL.setPower(0);
        flywheelR.setPower(0);
    }

    public void stopAll() {
        stopFlywheel();
        stopIntake();
    }

    public void topIntake() {
        runFlywheel(-1);
        runIntake(-0.5);
    }

}
