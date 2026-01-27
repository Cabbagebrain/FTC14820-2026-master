package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

@TeleOp
public class TagTrackTest extends LinearOpMode {
    private IMU imu;
    private Limelight3A limelight;
    private GamepadEx gamepadEx;
    private double x;
    private double y;
    private double rx;
    private ElapsedTime runtime;
    private double lastLoopTime;

    @Override
    public void runOpMode() throws InterruptedException {
        imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.pipelineSwitch(0);        // force AprilTag pipeline
        limelight.start(); // This tells Limelight to start looking!
        Supplier<LLResult> llSupplier = () -> limelight.getLatestResult();

        runtime = new ElapsedTime();
        lastLoopTime = runtime.seconds();
        DoubleSupplier deltaTimeSupplier = () -> {
            double currentTime = runtime.seconds();
            double dt = currentTime - lastLoopTime;
            lastLoopTime = currentTime;
            return dt;
        };

        MecanumDrivetrain drivetrain = new MecanumDrivetrain(hardwareMap);
        RampSubsystem ramp = new RampSubsystem(hardwareMap);
        ShintakeSubsystem shintake = new ShintakeSubsystem(hardwareMap);
        AprilTag april = new AprilTag(drivetrain, imu);
        gamepadEx = new GamepadEx(gamepad1);
        TagTrackDrive tagTrackDrive = new TagTrackDrive(drivetrain, april, telemetry, () -> gamepad1.left_stick_x, () -> gamepad1.left_stick_y, llSupplier, deltaTimeSupplier);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if (gamepad1.left_stick_button) {
                imu.resetYaw();
            }

            if (gamepad1.a) {
                if (!tagTrackDrive.isScheduled()) {
                    CommandScheduler.getInstance().schedule(tagTrackDrive);
                }
            } else {
                CommandScheduler.getInstance().cancel(tagTrackDrive);
                // manual control
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
                } else if (gamepad1.right_trigger == 0 && !gamepad1.right_bumper) {
                    shintake.stopFlywheel();
                }

                if (gamepad1.left_trigger > 0) {
                    shintake.runIntake(.75);
                } else if (gamepad1.left_trigger == 0 && !gamepad1.right_bumper) {
                    shintake.stopIntake();
                }

                if (gamepad1.right_bumper) {
                    shintake.topIntake();
                } else if (gamepad1.left_trigger == 0 && gamepad1.right_trigger == 0) {
                    shintake.stopAll();
                }
            }

            CommandScheduler.getInstance().run();
            LLResult result = llSupplier.get();

            telemetry.addData("Fiducial count: ", result.getFiducialResults().size());
            telemetry.addData("Valid: ", result.isValid());
            telemetry.addData("Pipeline: ", result.getPipelineIndex());
            telemetry.addData("Latency: ", result.getParseLatency());

            if (!result.getFiducialResults().isEmpty()) {
                telemetry.addData("Target X", result.getTx());
                telemetry.addData("Target Y", result.getTy());
                telemetry.addData("Target Area", result.getTa()); //percent of space an april tag takes up in the screen
            }

            telemetry.update();

        }
    }
