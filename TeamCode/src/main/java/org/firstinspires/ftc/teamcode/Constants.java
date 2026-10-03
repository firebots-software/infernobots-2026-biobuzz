package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;

public class Constants {
    public static class Conversions {
        public static double metersToInches = 39.3701;
    }

    public static class FieldSize {
        public static double yInches = 144;
        public static double xInches = 144;
    }

    public static class OdometryWheels {
        public static double parOffsetMM = 0;
        public static double perpOffsetMM = 0;
    }

    public static class Target {
        // High goal or target location on the field
        public static Pose position = new Pose(0, 0, 0);
    }

    public static class Shooter {
        // Empty Intermaps for distance -> shooting speed and distance -> time of flight
        public static Intermap distanceToSpeed = new Intermap();
        public static Intermap distanceToTimeOfFlight = new Intermap();

        // Fallback values if Intermaps are unpopulated
        public static double defaultSpeed = 2000.0; // ticks/sec or RPM
        public static double defaultTimeOfFlight = 0.5; // seconds
    }
}
