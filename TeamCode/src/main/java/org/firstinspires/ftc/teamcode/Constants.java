package org.firstinspires.ftc.teamcode;

public class Constants {
    public static class DriveConstants{
        //PID TUNING CONSTANTS
        public static final double HEADING_KP = 0.0107;
        public static final double HEADING_KI = 0.0014;
        public static final double HEADING_KD = 0.0019;
        public static final double HEADING_KF = 0.05;
        public static final double DRIVE_KP = 0;
        public static final double DRIVE_KI = 0;
        public static final double DRIVE_KD = 0;

        //anti-windup: max output [0.0, 1.0]
        public static final double MAX_PID_OUTPUT = 1.0;
    }
    public static class AprilConstants {
        //TODO: calibrate these
        public static final double REFERENCE_DISTANCE = 72.0;
        public static final double REFERENCE_TA = 0.9078;
        public static double TURN_TOLERANCE_DEG = 1.0;     // stop turning when within 1°
        public static double AREA_TOLERANCE = 0.05;        // acceptable ± range for tag area

        public static double MIN_TURN_POWER = 0.08;
        public static double MAX_TURN_POWER = 0.4;

        public static double MIN_DRIVE_POWER = 0.08;
        public static double MAX_DRIVE_POWER = 0.5;

        // Adjust these numbers to suit your robot.
        //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
        //  applied to the drive motors to correct the error.
        //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
        public static final double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". e.g. Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
        public static final double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  e.g. Ramp up to 37% power at a 25 degree Yaw error.   (0.375 / 25.0)
        public static final double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  e.g. Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    }
}
