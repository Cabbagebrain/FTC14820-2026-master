package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

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

        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!

        MecanumDrivetrain drivetrain = new MecanumDrivetrain(hardwareMap);
        RampSubsystem ramp = new RampSubsystem(hardwareMap);
        ShintakeSubsystem shintake = new ShintakeSubsystem(hardwareMap);
        AprilTag april = new AprilTag(drivetrain, imu);

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


            LLResult result = limelight.getLatestResult();

            if (result.isValid()) {
                telemetry.addData("Is Valid?", result.isValid());
                telemetry.addData("Target X", result.getTx());
                telemetry.addData("Target Y", result.getTy());
                telemetry.addData("Target Area", result.getTa()); //percent of space an april tag takes up in the screen
            } else {
                telemetry.addData("Limelight" , "No targets");
            }

            telemetry.update();

            boolean isAligning = false;

            if(result.isValid()) {
                if (gamepad1.a) {
                    april.alignToShoot(result, deltaTime);
                    isAligning = true;
                }
                if (gamepad1.b) {
                    april.faceTag(result, deltaTime);
                    isAligning = true;
                }
            }

            if (!isAligning) {
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
            }
        }
    }
    private double getHeadingDegrees() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }
}
