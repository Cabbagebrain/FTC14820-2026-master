package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class MecanumDrivetrain {
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private  DcMotor backRight;
    private Gamepad gamepad1;
    private double flPower;
    private double frPower;
    private double blPower;
    private double brPower;

    public MecanumDrivetrain(HardwareMap hardwareMap) {
        //retrieve motors from hardware map
        frontLeft = hardwareMap.get(DcMotor.class, "frontleft"); //port 0
        frontRight = hardwareMap.get(DcMotor.class, "frontright"); //port 1
        backLeft = hardwareMap.get(DcMotor.class, "backleft"); //port 2
        backRight = hardwareMap.get(DcMotor.class, "backright"); //port 3

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setPower(IMU imu, double x, double y, double rx) {
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        rx = -rx;
        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
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

    //sets robot at halfspeed when a button is held
    public void slowDrive() {
        frontLeft.setPower(flPower / 2);
        frontRight.setPower(frPower / 2);
        backLeft.setPower(blPower / 2);
        backRight.setPower(brPower / 2);
    }


}
