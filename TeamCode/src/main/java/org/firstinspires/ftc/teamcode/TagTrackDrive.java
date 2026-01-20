package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Constants.AprilConstants.DESIRED_TA;
import static org.firstinspires.ftc.teamcode.Constants.DriveConstants.HEADING_KD;
import static org.firstinspires.ftc.teamcode.Constants.DriveConstants.HEADING_KI;
import static org.firstinspires.ftc.teamcode.Constants.DriveConstants.HEADING_KP;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.DoubleSupplier;

public class TagTrackDrive extends CommandBase {
    private MecanumDrivetrain drivetrain;
    private AprilTag april;
    private Telemetry telemetry;
    private DoubleSupplier strafeSupplier;
    private DoubleSupplier driveSupplier;

    private LLResult ll;
    private PIDController headingPID;
    private double deltaTime;
    private static final double STOPPING_DISTANCE_THRESHOLD = 6.0;

    public TagTrackDrive (MecanumDrivetrain drive,
                          AprilTag april,
                          Telemetry telemetry,
                          DoubleSupplier strafeSupplier,
                          DoubleSupplier driveSupplier,
                          LLResult ll,
                          double deltaTime) {
        drivetrain = drive;
        this.april = april;
        this.telemetry = telemetry;
        this.strafeSupplier = strafeSupplier;
        this.driveSupplier = driveSupplier;
        this.ll = ll;
        this.deltaTime = deltaTime;
        headingPID = new PIDController(HEADING_KP, HEADING_KI, HEADING_KD);
        headingPID.setAngleTarget(0);

        addRequirements((Subsystem) drivetrain);
    }

    @Override
    public void execute() {
        double strafe = strafeSupplier.getAsDouble();
        double drive = driveSupplier.getAsDouble();
        double turn = 0;
        double distance = 0; //placeholder

        if (ll.isValid()) {
            double tx = ll.getTx();
            double ta = ll.getTa();

            if (Math.abs(tx) > 1) {
                turn = -headingPID.calculateHeadingOutput(tx, deltaTime);
            }


            // Assist forward motion only if driver isn't moving
            if (Math.abs(drive) < 0.05) {
                if (ta > DESIRED_TA + STOPPING_DISTANCE_THRESHOLD) {
                    drive = -0.4;
                } else if (ta < DESIRED_TA - STOPPING_DISTANCE_THRESHOLD) {
                    drive = 0.4;
                } else {
                    drive = 0.0;
                }
            }


            telemetry.addData("tx", tx);
            telemetry.addData("ta", ta);
            telemetry.addData("strafe", strafe);
            telemetry.addData("drive", drive);
            telemetry.addData("turn", turn);
        } else {
            telemetry.addData("Limelight", "No targets");
        }
        drivetrain.setPower(null, strafe, drive, turn);
        telemetry.update();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}
