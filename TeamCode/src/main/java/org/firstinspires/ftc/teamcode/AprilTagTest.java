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
import static org.firstinspires.ftc.teamcode.Constants.DriveConstants.HEADING_KD;
import static org.firstinspires.ftc.teamcode.Constants.DriveConstants.HEADING_KF;
import static org.firstinspires.ftc.teamcode.Constants.DriveConstants.HEADING_KI;
import static org.firstinspires.ftc.teamcode.Constants.DriveConstants.HEADING_KP;

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
    // Adjust these numbers to suit your robot.
    //todo: find these values
    final double DESIRED_TX = 0.0;     // centered
    final double DESIRED_TA = 2.5;     // tune this experimentally

    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.5;   //  Clip the strafing speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)


    @Override
    public void runOpMode() throws InterruptedException {
        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);

        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!

        MecanumDrivetrain drivetrain = new MecanumDrivetrain(hardwareMap);
        RampSubsystem ramp = new RampSubsystem(hardwareMap);
        ShintakeSubsystem shintake = new ShintakeSubsystem(hardwareMap);


        //todo: fine tune these constants & put into the Constants.java class
        PIDController strafePID = new PIDController(0.035, 0.0, 0.003);
        PIDController drivePID  = new PIDController(0.06,  0.0, 0.004);
        PIDController headingPID= new PIDController(HEADING_KP, HEADING_KI, HEADING_KD);
        strafePID.setDistTarget(0.0);        // tx centered
        drivePID.setDistTarget(DESIRED_TA);  // desired tag area
        headingPID.setAngleTarget(getHeadingDegrees()); // lock heading

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

            if (result.isValid() && gamepad1.a) {
                double txError = result.getTx() - DESIRED_TX;
                double taError = DESIRED_TA - result.getTa(); // inverted on purpose

                // STRAFE PID (tx)
                double strafePower = strafePID.calculateDriveOutput(result.getTx(), deltaTime);
                // DRIVE PID (ta)
                double drivePower = drivePID.calculateDriveOutput(result.getTa(), deltaTime);
                // HEADING PID
                double turnPower = headingPID.calculateHeadingOutput(getHeadingDegrees(), deltaTime);

                // Clamp outputs
                strafePower = Range.clip(strafePower, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
                drivePower  = Range.clip(drivePower,  -MAX_AUTO_SPEED,  MAX_AUTO_SPEED);
                turnPower   = Range.clip(turnPower,   -MAX_AUTO_TURN,   MAX_AUTO_TURN);

                drivetrain.setPower(imu, strafePower, drivePower, turnPower);
            } else {
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

            if (result.isValid()) {
                telemetry.addData("Is Valid?", result.isValid());
                telemetry.addData("Target X", result.getTx());
                telemetry.addData("Target Y", result.getTy());
                telemetry.addData("Target Area", result.getTa()); //percent of space an april tag takes up in the screen
            } else {
                telemetry.addData("Limelight" , "No targets");
            }

            telemetry.update();
        }
    }
    private double getHeadingDegrees() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }
}
