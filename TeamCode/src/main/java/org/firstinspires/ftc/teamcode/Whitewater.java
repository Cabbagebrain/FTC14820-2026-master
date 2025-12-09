package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Constants.DriveConstants.KD;
import static org.firstinspires.ftc.teamcode.Constants.DriveConstants.KF;
import static org.firstinspires.ftc.teamcode.Constants.DriveConstants.KI;
import static org.firstinspires.ftc.teamcode.Constants.DriveConstants.KP;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Roadrunner.drive.SampleMecanumDrive;

@Autonomous (name = "Whitewater", group = "Autonomous")
public class Whitewater extends LinearOpMode {
    private IMU imu;
    private double lastLoopTime;

    @Override
    public void runOpMode() throws InterruptedException {
        //Retrieve the IMU from the hardware map
        imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);

        //init limelight
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!

        //init PID
        PIDController pidController = new PIDController(KP, KI, KD);

        //init subsystems
        MecanumDrivetrain drivetrain = new MecanumDrivetrain(hardwareMap);
        RampSubsystem ramp = new RampSubsystem(hardwareMap);
        ShintakeSubsystem shintake = new ShintakeSubsystem(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        ElapsedTime runtime = new ElapsedTime();
        waitForStart();

        if (isStopRequested()) return;

        if (opModeIsActive()) {
            double currentTime = runtime.seconds();
            double deltaTime = currentTime - lastLoopTime;
            lastLoopTime = currentTime;

            double pidOutput = pidController.calculateOutput(getHeadingDegrees(), deltaTime);
            // APPLY FEEDFORWARD (KF)
            double correction = pidOutput;
            // Only apply Feedforward if the PID is commanding movement above a small threshold (0.01).
            if (Math.abs(pidOutput) > 0.01) {
                // Add KF in the direction of the correction (using Math.copySign).
                correction += Math.copySign(KF, pidOutput);
            }


            ramp.dropRamp();
            drivetrain.back(.75, 1.92);

            shintake.runFlywheel(.65);
            shintake.runIntake(1);
            sleep(4000);
            ramp.liftRamp();
            sleep(1000);
            shintake.stopFlywheel();
            shintake.stopIntake();
            ramp.dropRamp();



            //red
            drivetrain.turnLeft(.75, .5);
            drivetrain.forward(.75, .4);


            /*
            //blue
            drivetrain.turnRight(.75, .5);
            drivetrain.forward(.75, .4);

             */


            //drivetrain.forward(.75, 1);
        }
    }
    private double getHeadingDegrees() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }
}
