package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.teamcode.Roadrunner.drive.StandardTrackingWheelLocalizer.encoderTicksToInches;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Roadrunner.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.Roadrunner.util.Encoder;

import java.util.ArrayList;
import java.util.List;

@TeleOp
public class LiveDrivePIDTuner extends LinearOpMode {
    private IMU imu;
    private double lastLoopTime;
    private double x;
    private double y;
    private double rx;
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private StandardTrackingWheelLocalizer localizer;
    private List<Integer> lastPositions = new ArrayList<>();
    private List<Integer> lastVels = new ArrayList<>();

    // PID Coefficients
    private double Kp = 0.01;
    private double Ki = 0;
    private double Kd = 0;

    //tuning step size
    private final double delta = 0.0001;

    @Override
    public void runOpMode() throws InterruptedException {
        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        //init PID
        PIDController pidController = new PIDController(Kp, Ki, Kd);
        pidController.setDistTarget(24);

        //init tracking wheel
        localizer = new StandardTrackingWheelLocalizer(hardwareMap, lastPositions, lastVels);

        // Motor initialization
        frontLeft = hardwareMap.get(DcMotor.class, "frontleft");
        frontRight = hardwareMap.get(DcMotor.class, "frontright");
        backLeft = hardwareMap.get(DcMotor.class, "backleft");
        backRight = hardwareMap.get(DcMotor.class, "backright");

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        ElapsedTime runtime = new ElapsedTime();
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double currentTime = runtime.seconds();
            double deltaTime = currentTime - lastLoopTime;
            lastLoopTime = currentTime;

            double currentDistance = getRobotDistance();
            double pidOutput = pidController.calculateDriveOutput(encoderTicksToInches(currentDistance), deltaTime);

            double fl = pidOutput;
            double fr = pidOutput;
            double bl = pidOutput;
            double br = pidOutput;

            double max = Math.max(Math.abs(fl), Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br))));
            if (max > 1.0) {
                fl /= max;
                fr /= max;
                bl /= max;
                br /= max;
            }

            if (gamepad1.dpad_up) {
                Kp += delta;
                sleep(150);
            }
            if (gamepad1.dpad_down) {
                Kp = Math.max(0, Kp - delta);
                sleep(150);
            }
            if (gamepad1.dpad_right) {
                Ki += delta;
                sleep(150);
            }
            if (gamepad1.dpad_left) {
                Ki = Math.max(0, Ki - delta);
                sleep(150);
            }
            if (gamepad1.y) {
                Kd += delta;
                sleep(150);
            }
            if (gamepad1.b) {
                Kd = Math.max(0, Kd - delta);
                sleep(150);
            }

            pidController.setPID(Kp, Ki, Kd);  // Update PID with new values


            if (gamepad1.a) {
                frontLeft.setPower(fl);
                frontRight.setPower(fr);
                backLeft.setPower(bl);
                backRight.setPower(br);
            }

            // Telemetry
            telemetry.addData("Target Distance (in)", pidController.getTarget());
            telemetry.addData("Current Distance (in)", currentDistance);
            telemetry.addData("Current Error (in)", currentDistance/pidController.getTarget());
            telemetry.addData("PID Output", pidOutput);
            telemetry.addData("Kp", Kp);
            telemetry.addData("Ki", Ki);
            telemetry.addData("Kd", Kd);
            telemetry.update();
        }
    }

    // Compute robot forward distance as average of left/right tracking wheels
    private double getRobotDistance() {
        List<Double> wheelPositions = localizer.getWheelPositions();
        double left = wheelPositions.get(0);
        double right = wheelPositions.get(1);
        return (left + right) / 2.0; // average distance in inches
    }
}
