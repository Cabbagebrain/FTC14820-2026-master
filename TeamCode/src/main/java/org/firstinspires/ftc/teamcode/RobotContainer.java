package org.firstinspires.ftc.teamcode;;

import static org.firstinspires.ftc.teamcode.Constants.DriveConstants.KD;
import static org.firstinspires.ftc.teamcode.Constants.DriveConstants.KI;
import static org.firstinspires.ftc.teamcode.Constants.DriveConstants.KP;
import static org.firstinspires.ftc.teamcode.Constants.DriveConstants.TARGET_ANGLE;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp
public class RobotContainer extends LinearOpMode {
    private IMU imu;
    private double robotYaw;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private ElapsedTime runtime;
    private double lastLoopTime;
    private double x;
    private double y;
    private double rx;
    private PIDController pidController;
    private Limelight3A limelight;
    private LLResult result;

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

        //init limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!

        //init PID
        pidController = new PIDController(KP, KI, KD);
        pidController.setTarget(TARGET_ANGLE);

        //init subsystems
        MecanumDrivetrain drivetrain = new MecanumDrivetrain(hardwareMap);

        runtime = new ElapsedTime();
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
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x + pidOutput;

            if (gamepad1.options) {
                imu.resetYaw();
            }

            drivetrain.setPower(imu, x, y, rx);
            drivetrain.drive();

            result = limelight.getLatestResult();

            if (result.isValid() && result != null) {
                telemetry.addData("Target X", result.getTx());
                telemetry.addData("Target Y", result.getTy()); //
                telemetry.addData("Target Area", result.getTa()); //percent of space an april tag takes up in the screen

                Pose3D botpose = result.getBotpose();
                if (botpose != null) {
                    telemetry.addData("Bot Location", "(" + botpose.getPosition().x + ", " + botpose.getPosition().y + ")");
                }
            } else {
                telemetry.addData("Limelight" , "No targets");
            }
            telemetry.addData("PID Output", pidOutput);

            telemetry.update();
        }
    }

    //returns current angle heading, normalized to {-180, 180]
    private double getHeadingDegrees() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }
}
