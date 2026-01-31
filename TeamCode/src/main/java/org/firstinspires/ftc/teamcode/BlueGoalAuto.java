package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Constants.DriveConstants.HEADING_KD;
import static org.firstinspires.ftc.teamcode.Constants.DriveConstants.HEADING_KF;
import static org.firstinspires.ftc.teamcode.Constants.DriveConstants.HEADING_KI;
import static org.firstinspires.ftc.teamcode.Constants.DriveConstants.HEADING_KP;

import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Roadrunner.trajectorysequence.TrajectorySequence;

import java.util.Vector;

@Autonomous (name = "Blue Goal Auto", group = "Autonomous")
public class BlueGoalAuto extends LinearOpMode {
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
        PIDController pidController = new PIDController(HEADING_KP, HEADING_KI, HEADING_KD);

        //init subsystems
        MecanumDrivetrain drivetrain = new MecanumDrivetrain(hardwareMap);
        RampSubsystem ramp = new RampSubsystem(hardwareMap);
        ShintakeSubsystem shintake = new ShintakeSubsystem(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        Pose2d initialPose = new Pose2d(-41, 47, toRadians(45));
        drive.setPoseEstimate(initialPose);

         TrajectorySequence start1= drive.trajectorySequenceBuilder(initialPose)
                .back(64.5)
                .build();

        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(start1.end())
                .turn(toRadians(80))
                .forward(40.0)
                .back(40.0)
                .build();

        TrajectorySequence traj3 = drive.trajectorySequenceBuilder(traj2.end())
                .turn(toRadians(-84))
                .forward(10)
                .build();


        waitForStart();
        if (isStopRequested()) return;

        if(opModeIsActive()) {
            shintake.runIntake(1);
            shintake.runFlywheel(.67);
            Thread.sleep(3000);
            ramp.liftRamp();
            Thread.sleep(3000);
            //shintake.stopAll();
            ramp.dropRamp();

            drive.followTrajectorySequenceAsync(traj2);
            while (opModeIsActive() && !isStopRequested()&& drive.isBusy()) {
                drive.update();
            }

            ramp.liftRamp();
            Thread.sleep(3000);
            ramp.dropRamp();
            shintake.stopAll();

            drive.followTrajectorySequenceAsync(traj3);
            while (opModeIsActive() && !isStopRequested()&& drive.isBusy()) {
                drive.update();
            }
        }
    }
    private double getHeadingDegrees() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }
}
