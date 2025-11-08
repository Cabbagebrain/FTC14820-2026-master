package org.firstinspires.ftc.teamcode;

public class Constants {
    public static class DriveConstants{
        //PID TUNING CONSTANTS
        public static final double KP = 0.0107;
        public static final double KI = 0.0014;
        public static final double KD = 0.0019;
        public static final double KF = 0.05;

        //anti-windup: max output [0.0, 1.0]
        public static final double MAX_PID_OUTPUT = 1.0;
    }
    public static class AprilConstants {
        //TODO: calibrate these
        public static final double REFERENCE_DISTANCE = 0.0;
        public static final double REFERENCE_TA = 0.0;
    }
}
