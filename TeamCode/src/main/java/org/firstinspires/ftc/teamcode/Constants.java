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
        public static final double DESIRED_TX = 0.0;     // centered
        public static final double DESIRED_TA = 2.5;     // tune this experimentally

        public static final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
        public static final double MAX_AUTO_STRAFE= 0.5;   //  Clip the strafing speed to this max value (adjust for your robot)
        public static final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)

    }
}
