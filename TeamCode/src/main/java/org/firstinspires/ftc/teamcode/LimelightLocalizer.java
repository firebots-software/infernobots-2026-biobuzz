package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@Configurable
public class LimelightLocalizer {

    // BIOBUZZ field dimensions (inches)
    public static double FIELD_LENGTH_IN = 144.0;
    public static double FIELD_WIDTH_IN = 144.0;

    // Position calibration: tune on the actual field
    public static double X_NUDGE = Constants.RobotOffSet.X_Nudge;
    public static double Y_NUDGE = Constants.RobotOffSet.Y_Nudge;
    public static double X_SCALE = Constants.RobotOffSet.X_Scale;
    public static double Y_SCALE = Constants.RobotOffSet.Y_Scale;

    private static final double METERS_TO_INCHES = 39.3701;

    private final Limelight3A limelight;

    public LimelightLocalizer(Limelight3A limelight) {
        this.limelight = limelight;
    }

    public void start() {
        limelight.start();
    }

    public void stop() {
        limelight.stop();
    }

    public Pose3D getLimelightPose() {
        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            return null;
        }

        return result.getBotpose();
    }

    public Pose getCorrectedPoseInInches() {
        Pose3D llOutput = getLimelightPose();

        if (llOutput == null) {
            return null;
        }

        Pose limelightPose = new Pose(
                llOutput.getPosition().x * METERS_TO_INCHES,
                llOutput.getPosition().y * METERS_TO_INCHES,
                llOutput.getOrientation().getYaw(AngleUnit.RADIANS),
                FTCCoordinates.INSTANCE
        );

        double heading = limelightPose.getHeading();
        heading -= Math.PI / 2.0;

        if (heading < 0) {
            heading += Math.PI * 2.0;
        }

        // Convert Limelight coordinates to field-centered coordinates.
        double x = limelightPose.getY() + FIELD_WIDTH_IN / 2.0;
        double y = -limelightPose.getX() + FIELD_LENGTH_IN / 2.0;

        // Apply calibration.
        x = x * X_SCALE + X_NUDGE;
        y = y * Y_SCALE + Y_NUDGE;

        return new Pose(x, y, heading);
    }

    public void telemetrizeLimelightPose() {
        Pose3D pose = getLimelightPose();

        if (pose != null) {
            org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion
                    .telemetry.addData("Limelight Pose", pose.toString());
        }
    }

    public void setPipeline(int pipeline) {
        limelight.pipelineSwitch(pipeline);
    }

    public Limelight3A getLimelight() {
        return limelight;
    }
}

