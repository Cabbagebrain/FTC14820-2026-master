package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@TeleOp
public class RobotContainer extends LinearOpMode {
    private IMU imu;
    private double robotYaw;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private ElapsedTime runtime;
    private double x;
    private double y;
    private double rx;
    private Limelight3A limelight;
    private LLResult result;

    @Override
    public void runOpMode() throws InterruptedException {
        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);


        //retrieve motors from hardware map
        frontLeft = hardwareMap.get(DcMotor.class, "front_left");
        frontRight = hardwareMap.get(DcMotor.class, "front_right");
        backLeft = hardwareMap.get(DcMotor.class, "back_left");
        backRight = hardwareMap.get(DcMotor.class, "back_right");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        //init subsystems
        MecanumDrivetrain drivetrain = new MecanumDrivetrain(frontLeft, frontRight, backLeft, backRight);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!

        runtime = new ElapsedTime();
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if(gamepad1.left_stick_button) {
                imu.resetYaw();
            }

            drivetrain.setPower(gamepad1, imu, x, y, rx);
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

            telemetry.update();
        }
    }
}
