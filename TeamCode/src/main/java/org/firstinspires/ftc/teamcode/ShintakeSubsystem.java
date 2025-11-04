package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ShintakeSubsystem {
    private DcMotor intake;
    private DcMotor flywheel;
    public ShintakeSubsystem(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotor.class, "intake"); //port 0
        flywheel = hardwareMap.get(DcMotor.class, "flywheel"); //port 1

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void runIntake(double power) {
        intake.setPower(power);
    }

    public void stopIntake() {
        intake.setPower(0);
    }

    public void runFlywheel(double power) {
        flywheel.setPower(power);
    }

    public void stopFlywheel() {
        flywheel.setPower(0);
    }
}
