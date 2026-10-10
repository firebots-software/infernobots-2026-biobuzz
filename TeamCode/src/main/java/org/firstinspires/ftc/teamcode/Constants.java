package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
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

    public static class Transfer {
        public static double defaultPower = 1.0;
    }

    public static class RobotOffSet {
        public static double X_Nudge = 0;
        public static double Y_Nudge = 0;
        public static double X_Scale = 1;
        public static double Y_Scale = 1;
    }

    @Configurable
    public static class Limelight {
        public static double FIELD_LENGTH_IN = 144.0;
        public static double FIELD_WIDTH_IN = 144.0;
        public static double X_NUDGE = 0.0;
        public static double Y_NUDGE = 0.0;
        public static double X_SCALE = 1.0;
        public static double Y_SCALE = 1.0;
    }
}
