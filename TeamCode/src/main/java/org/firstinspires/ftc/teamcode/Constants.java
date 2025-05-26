package org.firstinspires.ftc.teamcode;

public class Constants {
    public static class DrivetrainConstants {
        public static final double gearing = 0.08804179566;// (50/8) * (20/75) * (21/76) * (13/68);
        public static final double wheelDiameter = (1.125 * 2) * 2.54; // cm
        public static final double motorMaxRpm = 6000;
        public static final double maxSpeed = motorMaxRpm * gearing * wheelDiameter * Math.PI * .7; // theoretical max speed minus 30%
        public static final double angleThreshold = 10; // TODO: find a more intelligent way to determine this threshold

        public static final double IMUOffset = 0; // Offset of internal IMU. Positive indicates counter-clockwise offset, ie. IMU is rotated clockwise relative to forward.

        public static final double frontEncoderOffset = 128;
        public static final double backEncoderOffset = 110;

        public static final double kDriveP = 0.02; //TODO: Tune drive PID constants
        public static final double kDriveI = 0.01;
        public static final double kDriveD = 0.0;

        public static final double kTurnP = 0.03; //TODO: Tune turn PID constants
        public static final double kTurnI = 0.0;
        public static final double kTurnD = 0.0;
    }
}
