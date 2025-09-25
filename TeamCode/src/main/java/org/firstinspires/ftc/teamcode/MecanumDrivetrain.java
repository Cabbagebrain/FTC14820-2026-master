package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class MecanumDrivetrain {
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private  DcMotor backRight;
    private Gamepad gamepad1;
    private double denominator;
    private double flPower;
    private double frPower;
    private double blPower;
    private double brPower;

    private double rotX;
    private double rotY;
    private double botHeading;
    public MecanumDrivetrain(DcMotor frontLeftMotor , DcMotor frontRightMotor, DcMotor backLeftMotor, DcMotor backRightMotor) {
        frontLeft = frontLeftMotor;
        frontRight = frontRightMotor;
        backLeft = backLeftMotor;
        backRight = backRightMotor;
    }

    public void setPower(Gamepad gamepad1, IMU imu, double x, double y, double rx) {
        botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        rx -= rx * 2;
        // Rotate the movement direction counter to the bot's rotation
        rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        flPower = (rotY - rotX + rx) / denominator;
        blPower = (rotY + rotX + rx) / denominator;
        frPower = (rotY - rotX - rx) / denominator;
        brPower = (rotY + rotX - rx) / denominator;
    }

    public void drive() {
        frontLeft.setPower(flPower);
        frontRight.setPower(frPower);
        backLeft.setPower(blPower);
        backRight.setPower(brPower);
    }


}
