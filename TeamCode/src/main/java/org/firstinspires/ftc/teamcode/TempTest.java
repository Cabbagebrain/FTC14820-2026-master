package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

public class TempTest extends LinearOpMode {
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
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.addData("Status", "Configuring Limelight...");
        telemetry.update();

        limelight.setPollRateHz(50);
        limelight.pipelineSwitch(0);
        limelight.start();

        sleep(1000); // Give it time to start

        telemetry.addData("Status", "Limelight started");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();

            // Comprehensive diagnostics
            telemetry.addData("1. Result null?", result == null);

            if (result != null) {
                telemetry.addData("2. Is Valid", result.isValid());
                telemetry.addData("3. Pipeline Index", result.getPipelineIndex());
                telemetry.addData("4. Pipeline Type", result.getPipelineType());
                telemetry.addData("5. Fiducial Count", result.getFiducialResults().size());
                telemetry.addData("6. Latency (ms)", result.getParseLatency());
                telemetry.addData("7. Timestamp", result.getTimestamp());

                if (!result.getFiducialResults().isEmpty()) {
                    telemetry.addData("8. TX", result.getTx());
                    telemetry.addData("9. TY", result.getTy());
                    telemetry.addData("10. TA", result.getTa());
                } else {
                    telemetry.addData("8. Status", "NO FIDUCIALS DETECTED");
                }
            } else {
                telemetry.addData("ERROR", "Result is NULL!");
            }

            telemetry.update();
            sleep(100); // Slow down updates
        }
    }
}
