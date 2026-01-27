/*package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.teamcode.Constants.DriveConstants.HEADING_KD;
import static org.firstinspires.ftc.teamcode.Constants.DriveConstants.HEADING_KI;
import static org.firstinspires.ftc.teamcode.Constants.DriveConstants.HEADING_KP;

import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.Roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Roadrunner.trajectorysequence.TrajectorySequence;

@Autonomous(name = "Red Goal Auto", group = "Autonomous")
public class RedGoalAuto {

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
        Pose2d initialPose = new Pose2d(41, 47, toRadians(45));
        drive.setPoseEstimate(initialPose);

        TrajectorySequence start1= drive.trajectorySequenceBuilder(initialPose)
                .back(64.5 )
                .build();

        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(start1.end())
                .turn(toRadians(-70))
                .build();

        TrajectorySequence forwards = drive.trajectorySequenceBuilder(traj1.end())
                .forward(10.0)
                .build();

        TrajectorySequence backwards = drive.trajectorySequenceBuilder(forwards.end())
                .back(10.0)
                .build();
TrajectorySequence traj2 = drive.trajectorySequenceBuilder(backwards.end())
                .turn(toRadians(70.0))
                .build();





if(opModeIsActive()) {
            double currentTime = runtime.seconds();
            double deltaTime = currentTime - lastLoopTime;
            lastLoopTime = currentTime;

            double pidOutput = pidController.calculateHeadingOutput(getHeadingDegrees(), deltaTime);
            // APPLY FEEDFORWARD (KF)
            double correction = pidOutput;
            // Only apply Feedforward if the PID is commanding movement above a small threshold (0.01).
            if (Math.abs(pidOutput) > 0.01) {
                // Add KF in the direction of the correction (using Math.copySign).
                correction += Math.copySign(HEADING_KF, pidOutput);
            }
            drive.followTrajectorySequenceAsync(start1);
            while (opModeIsActive() && !isStopRequested()&&drive.isBusy()) {
                drive.update();

                shintake.runIntake(1);
            shintake.runFlywheel(.60);
            Thread.sleep(5000);
            ramp.liftRamp();
            Thread.sleep(4000);

            ramp.dropRamp();

            drive.followTrajectorySequenceAsync(traj1);
            while (opModeIsActive() && !isStopRequested()&&drive.isBusy()) {
                drive.update();

            drive.followTrajectorySequenceAsync(forwards);
            while (opModeIsActive() && !isStopRequested()&&drive.isBusy()) {
                drive.update();

            }
            shintake.runIntake(1);
            Thread.sleep(4000);
            ramp.liftRamp();
            shintake.runFlywheel(.75);
            Thread.sleep(1000);
            shintake.stopAll();
            ramp.dropRamp();
 drive.followTrajectorySequenceAsync(traj2);
            while (opModeIsActive() && !isStopRequested()&& drive.isBusy()) {
                drive.update();
            }

            drive.followTrajectorySequenceAsync(forwards);
            while (opModeIsActive() && !isStopRequested()&& drive.isBusy()) {
                drive.update();
            }
            shintake.runIntake(1);
            drive.followTrajectorySequenceAsync(backwards);
            while (opModeIsActive() && !isStopRequested()&&drive.isBusy()) {
                drive.update();
            };
            drive.followTrajectorySequenceAsync(traj3);
            while (opModeIsActive() && !isStopRequested()&& drive.isBusy()) {
                drive.update();
            }

            shintake.runIntake(1);
            shintake.runFlywheel(.59);
           // Thread.sleep(5000);
            ramp.liftRamp();
            Thread.sleep(4000);
            ramp.dropRamp();
            shintake.stopAll();



        }

    }
*/