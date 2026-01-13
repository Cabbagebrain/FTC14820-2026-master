package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Constants.AprilConstants.AREA_TOLERANCE;
import static org.firstinspires.ftc.teamcode.Constants.AprilConstants.MAX_DRIVE_POWER;
import static org.firstinspires.ftc.teamcode.Constants.AprilConstants.MAX_TURN_POWER;
import static org.firstinspires.ftc.teamcode.Constants.AprilConstants.MIN_DRIVE_POWER;
import static org.firstinspires.ftc.teamcode.Constants.AprilConstants.MIN_TURN_POWER;
import static org.firstinspires.ftc.teamcode.Constants.AprilConstants.SPEED_GAIN;
import static org.firstinspires.ftc.teamcode.Constants.AprilConstants.TURN_GAIN;
import static org.firstinspires.ftc.teamcode.Constants.AprilConstants.TURN_TOLERANCE_DEG;

import static java.lang.Thread.sleep;

import com.qualcomm.hardware.limelightvision.LLResult;
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
        double rotX = x;
        double rotY = -y;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

        flPower = (rotY - rotX + rx) / denominator;
        blPower = (rotY + rotX + rx) / denominator;
        frPower = (rotY - rotX - rx) / denominator;
        brPower = (rotY + rotX - rx) / denominator;

        /*
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
         */
    }
    /*
    public void turn(IMU imu, double rx) {
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        rx = -rx;
        // Rotate the movement direction counter to the bot's rotation
        double rotX = 0;
        double rotY = 0;

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        flPower = (rotY - rotX + rx) / denominator;
        blPower = (rotY + rotX + rx) / denominator;
        frPower = (rotY - rotX - rx) / denominator;
        brPower = (rotY + rotX - rx) / denominator;
    }
    */

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
    public void stop() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public void forward(double power, double sec) throws InterruptedException {
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
        Thread.sleep((long) sec * 1000);
        stop();
    }

    public void back(double power, double sec) throws InterruptedException {
        frontLeft.setPower(-power);
        frontRight.setPower(-power);
        backLeft.setPower(-power);
        backRight.setPower(-power);
        Thread.sleep((long) sec * 1000);
        stop();
    }

    public void strafeLeft(double power, double sec) throws InterruptedException {
        frontLeft.setPower(-power);
        frontRight.setPower(-power);
        backLeft.setPower(power);
        backRight.setPower(-power);
        Thread.sleep((long) sec * 1000);
        stop();
    }

    public void strafeRight(double power, double sec) throws InterruptedException {
        frontLeft.setPower(power);
        frontRight.setPower(-power);
        backLeft.setPower(-power);
        backRight.setPower(power);
        Thread.sleep((long) sec * 1000);
        stop();
    }

    public void turnLeft(double power, double sec) throws InterruptedException {
        frontLeft.setPower(-power);
        backLeft.setPower(-power);
        frontRight.setPower(power);
        backRight.setPower(power);
        Thread.sleep((long) sec * 1000);
        stop();
    }

    public void turnRight(double power, double sec) throws InterruptedException {
        frontLeft.setPower(power);
        backLeft.setPower(power);
        frontRight.setPower(-power);
        backRight.setPower(-power);
        Thread.sleep((long) sec * 1000);
        stop();
    }


}
