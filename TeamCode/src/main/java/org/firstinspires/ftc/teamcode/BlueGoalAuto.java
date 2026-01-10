package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Constants.DriveConstants.HEADING_KD;
import static org.firstinspires.ftc.teamcode.Constants.DriveConstants.HEADING_KF;
import static org.firstinspires.ftc.teamcode.Constants.DriveConstants.HEADING_KI;
import static org.firstinspires.ftc.teamcode.Constants.DriveConstants.HEADING_KP;

import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
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

        /*

        //TODO: confirm values
        //Pose2d initialPose = new Pose2d(4.95, 4.75, toRadians(45);
        Pose2d initialPose = new Pose2d(63,0,toRadians(180));
        drive.setPoseEstimate(initialPose);

        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(initialPose)
                .back(35)
                .turn(toRadians(180))
                .build();

        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(initialPose)
                .turn(toRadians(180))
                .forward(35)
                .build();

        Vector2d endPos= new Vector2d(0,0);
        TrajectorySequence test = drive.trajectorySequenceBuilder(initialPose)
                .lineTo(endPos)
                .build();

         */

        ElapsedTime runtime = new ElapsedTime();
        waitForStart();

        if (isStopRequested()) return;

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

            //align to shoot
            drivetrain.back(.75, 1.92);
            pidController.setAngleTarget(-47);
            drivetrain.setPower(imu, 0, 0, correction);

            shintake.runFlywheel(1);
            shintake.runIntake(1);
            sleep(4000);
            ramp.liftRamp();
            sleep(1000);
            shintake.stopFlywheel();
            shintake.stopIntake();
            ramp.dropRamp();

            pidController.setAngleTarget(-90);
            drivetrain.setPower(imu, 0 ,0, correction);


            //pickup first row of balls
            drivetrain.strafeLeft(.75, .4);
            shintake.runIntake(1);
            drivetrain.forward(.5, .4);
            shintake.stopIntake();

            // go back to shooting zone
            drivetrain.back(.75,.4);
            drivetrain.strafeRight(.75,.4);
            pidController.setAngleTarget(-47);
            drivetrain.setPower(imu, 0, 0, correction);
            drivetrain.back(.5, .3);

            shintake.runFlywheel(1);
            shintake.runIntake(1);
            sleep(4000);
            ramp.liftRamp();
            sleep(1000);
            shintake.stopFlywheel();
            shintake.stopIntake();
            ramp.dropRamp();

            //drivetrain.strafeRight(.75, .5);
            /*
            drivetrain.back(.75,2.4);

            shintake.runFlywheel(-1);
            shintake.runIntake(-0.5);
            sleep(5000);

            shintake.stopFlywheel();
            sleep(1000);

            drivetrain.forward(.75, 2.4);
            shintake.runFlywheel(1);
            shintake.runIntake(1);
            sleep(4000);
            ramp.liftRamp();
            sleep(1000);
            shintake.stopFlywheel();
            shintake.stopIntake();
            ramp.dropRamp();
            drivetrain.back(.75,2.4);


            /*
            drive.followTrajectorySequenceAsync(test);
            while (opModeIsActive() && !isStopRequested() && drive.isBusy()) {
                drive.update();
            }
             */

            /*
            //auto routine
            shintake.runFlywheel(1);
            shintake.runIntake(1);
            sleep(4000);
            ramp.liftRamp();
            sleep(1000);
            shintake.stopFlywheel();
            shintake.stopIntake();
            drive.followTrajectorySequenceAsync(traj1);
            while (opModeIsActive() && !isStopRequested() && drive.isBusy()) {
                drive.update();
            }

            drive.followTrajectorySequenceAsync(traj2); //return to shoot
            while (opModeIsActive() && !isStopRequested() && drive.isBusy()) {
                drive.update();
            }

            shintake.runFlywheel(1);
            shintake.runIntake(1);
            sleep(4000);
            ramp.liftRamp();
            sleep(1000);
            shintake.stopFlywheel();
            shintake.stopIntake();

            drive.followTrajectorySequenceAsync(traj1); //return to loading zone
            while (opModeIsActive() && !isStopRequested()) {
                drive.update();
            }
             */
        }

    }
    private double getHeadingDegrees() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }
}
