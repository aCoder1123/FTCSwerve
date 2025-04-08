package org.firstinspires.ftc.teamcode;

public class Constants {
    public static class DrivetrainConstants {
        public static final double gearing = 0.08804179566;// (50/8) * (20/75) * (21/76) * (13/68);
        public static final double wheelSeparation = 46.39782165; // cm
        public static final double wheelDiameter = (1.125 * 2) * 2.54; // cm
        public static final double maxSpeed = 6000 * gearing * wheelDiameter * Math.PI * .7; // theoretical max speed minus 30%
        public static final double angleThreshold = 15; // TODO: find a more intelligent way to determine this threshold
        public static final double motorMaxRpm = 6000;

        public static final double frontEncoderOffset = 128;
        public static final double backEncoderOffset = 39.5;
    }
}
