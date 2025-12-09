package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Constants.AprilConstants.AREA_TOLERANCE;
import static org.firstinspires.ftc.teamcode.Constants.AprilConstants.MAX_DRIVE_POWER;
import static org.firstinspires.ftc.teamcode.Constants.AprilConstants.MAX_TURN_POWER;
import static org.firstinspires.ftc.teamcode.Constants.AprilConstants.MIN_DRIVE_POWER;
import static org.firstinspires.ftc.teamcode.Constants.AprilConstants.MIN_TURN_POWER;
import static org.firstinspires.ftc.teamcode.Constants.AprilConstants.REFERENCE_DISTANCE;
import static org.firstinspires.ftc.teamcode.Constants.AprilConstants.REFERENCE_TA;
import static org.firstinspires.ftc.teamcode.Constants.AprilConstants.TURN_GAIN;
import static org.firstinspires.ftc.teamcode.Constants.AprilConstants.TURN_TOLERANCE_DEG;
import static org.firstinspires.ftc.teamcode.Constants.AprilConstants.SPEED_GAIN;
import static org.firstinspires.ftc.teamcode.Constants.DriveConstants.KD;
import static org.firstinspires.ftc.teamcode.Constants.DriveConstants.KF;
import static org.firstinspires.ftc.teamcode.Constants.DriveConstants.KI;
import static org.firstinspires.ftc.teamcode.Constants.DriveConstants.KP;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp
public class AprilTagTest extends LinearOpMode{
    private IMU imu;
    private double lastLoopTime;
    private double x;
    private double y;
    private double rx;

    @Override
    public void runOpMode() throws InterruptedException {
        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                //TODO make this accurate to the new hub orientation
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);

        //init limelight
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!

        //init PID
        PIDController pidController = new PIDController(KP, KI, KD);
        pidController.setTarget(90);

        //init subsystems
        MecanumDrivetrain drivetrain = new MecanumDrivetrain(hardwareMap);
        RampSubsystem ramp = new RampSubsystem(hardwareMap);
        ShintakeSubsystem shintake = new ShintakeSubsystem(hardwareMap);

        ElapsedTime runtime = new ElapsedTime();
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double currentTime = runtime.seconds();
            double deltaTime = currentTime - lastLoopTime;
            lastLoopTime = currentTime;

            if(gamepad1.left_stick_button) {
                imu.resetYaw();
            }

            double pidOutput = pidController.calculateOutput(getHeadingDegrees(), deltaTime);
            // APPLY FEEDFORWARD (KF)
            double correction = pidOutput;
            // Only apply Feedforward if the PID is commanding movement above a small threshold (0.01).
            if (Math.abs(pidOutput) > 0.01) {
                // Add KF in the direction of the correction (using Math.copySign).
                correction += Math.copySign(KF, pidOutput);
            }

            y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            x = gamepad1.left_stick_x;
            rx = gamepad1.right_stick_x;

            drivetrain.setPower(imu, x, y, rx);
            if (gamepad1.left_bumper) {
                drivetrain.slowDrive();
            } else {
                drivetrain.drive();
            }

            if (gamepad1.dpad_down) {
                ramp.dropRamp();
            }
            if (gamepad1.dpad_up) {
                ramp.liftRamp();
            }

            if (gamepad1.right_trigger > 0) {
                shintake.runFlywheel(.7);
            } else if (gamepad1.right_trigger == 0 && !gamepad1.right_bumper){
                shintake.stopFlywheel();
            }

            if (gamepad1.left_trigger > 0) {
                shintake.runIntake(.75);
            } else if (gamepad1.left_trigger == 0 && !gamepad1.right_bumper) {
                shintake.stopIntake();
            }

            if (gamepad1.right_bumper) {
                shintake.topIntake();
            } else if (gamepad1.left_trigger == 0 && gamepad1.right_trigger == 0){
                shintake.stopAll();
            }

            LLResult result = limelight.getLatestResult();
            if (result.isValid()) {
                telemetry.addData("Target X", result.getTx());
                telemetry.addData("Target Y", result.getTy());
                telemetry.addData("Target Area", result.getTa()); //percent of space an april tag takes up in the screen
            } else {
                telemetry.addData("Limelight" , "No targets");
            }
            telemetry.addData("PID Output", pidOutput);
            telemetry.addData("Current Angle", getHeadingDegrees());

            telemetry.update();
        }
    }
    private double getHeadingDegrees() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }
    private double distanceToArea(double desiredDistance) {
        return REFERENCE_DISTANCE * Math.pow(REFERENCE_TA / desiredDistance, 2.0);
    }
}
