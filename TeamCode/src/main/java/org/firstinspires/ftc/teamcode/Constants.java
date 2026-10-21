package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

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

    public static class Transfer {
        public static String motorName = "transfer";
        public static DcMotorSimple.Direction motorDirection = DcMotorSimple.Direction.FORWARD;
        public static DcMotor.ZeroPowerBehavior zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE;
        public static double transferPower = 0.8;
        public static double reversePower = -0.5;
    }

    public static class Intake {
        public static String motorName = "intake";
        public static DcMotorSimple.Direction motorDirection = DcMotorSimple.Direction.FORWARD;
        public static DcMotor.ZeroPowerBehavior zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE;
        public static double intakePower = 1.0;
        public static double outtakePower = -0.7;
    }
}
