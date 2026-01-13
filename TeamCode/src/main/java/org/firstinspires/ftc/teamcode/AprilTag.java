package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.teamcode.Constants.DriveConstants.HEADING_KD;
import static org.firstinspires.ftc.teamcode.Constants.DriveConstants.HEADING_KI;
import static org.firstinspires.ftc.teamcode.Constants.DriveConstants.HEADING_KP;
import static org.firstinspires.ftc.teamcode.Constants.AprilConstants.DESIRED_TA;
import static org.firstinspires.ftc.teamcode.Constants.AprilConstants.DESIRED_TX;
import static org.firstinspires.ftc.teamcode.Constants.AprilConstants.MAX_AUTO_SPEED;
import static org.firstinspires.ftc.teamcode.Constants.AprilConstants.MAX_AUTO_STRAFE;
import static org.firstinspires.ftc.teamcode.Constants.AprilConstants.MAX_AUTO_TURN;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class AprilTag {
    private MecanumDrivetrain drivetrain;
    private IMU imu;
    private PIDController strafePID;
    private PIDController drivePID;
    private PIDController headingPID;


    public AprilTag(MecanumDrivetrain drive, IMU eyeMyou) {
        drivetrain = drive;
        imu = eyeMyou;
        strafePID = new PIDController(0.035, 0.0, 0.003);
        drivePID  = new PIDController(0.06,  0.0, 0.004);
        headingPID= new PIDController(HEADING_KP, HEADING_KI, HEADING_KD);
    }

    public void faceTag(LLResult result, double deltaTime) {
        PIDController headingPID = new PIDController(HEADING_KP, HEADING_KI, HEADING_KD);
        double currentHeading = getHeadingDegrees(imu);
        double desiredHeading = currentHeading + result.getTx();
        headingPID.setAngleTarget(desiredHeading);

        double rx = headingPID.calculateHeadingOutput(currentHeading, deltaTime);
        rx = Range.clip(rx, -0.3, 0.3);
        drivetrain.setPower(imu, 0, 0, rx);
        if (Math.abs(headingPID.getTarget() - getHeadingDegrees(imu)) < 1.0) {
            drivetrain.setPower(imu, 0, 0, 0);
        }
    }

    //TODO: create lineup
    public void alignToShoot(LLResult result, double deltaTime) {
        strafePID.setDistTarget(0.0);        // tx centered
        drivePID.setDistTarget(DESIRED_TA);  // desired tag area
        headingPID.setAngleTarget(getHeadingDegrees(imu)); // lock heading

        double txError = result.getTx() - DESIRED_TX;
        double taError = DESIRED_TA - result.getTa(); // inverted on purpose

        // STRAFE PID (tx)
        double strafePower = strafePID.calculateDriveOutput(result.getTx(), deltaTime);
        // DRIVE PID (ta)
        double drivePower = drivePID.calculateDriveOutput(result.getTa(), deltaTime);
        // HEADING PID
        double turnPower = headingPID.calculateHeadingOutput(getHeadingDegrees(imu), deltaTime);

        // Clamp outputs
        strafePower = Range.clip(strafePower, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
        drivePower  = Range.clip(drivePower,  -MAX_AUTO_SPEED,  MAX_AUTO_SPEED);
        turnPower   = Range.clip(turnPower,   -MAX_AUTO_TURN,   MAX_AUTO_TURN);

        drivetrain.setPower(imu, strafePower, drivePower, turnPower);
    }
    private double getHeadingDegrees(IMU imu) {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }

}
