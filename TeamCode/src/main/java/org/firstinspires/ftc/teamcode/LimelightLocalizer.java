package org.firstinspires.ftc.teamcode;

import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class LimelightLocalizer {
    private final Limelight3A limelight;

    public LimelightLocalizer(Limelight3A limelight) {
        this.limelight = limelight;
    }

    public void start() {
        this.limelight.start();
    }

    public void stop() {
        this.limelight.stop();
    }

    public Pose3D getLimelightPose() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            return result.getBotpose();
        }
        return null;
    }

    public void telemetrizeLimelightPose(Telemetry telemetry) {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid() && telemetry != null) {
            Pose3D botpose = result.getBotpose();
            telemetry.addData("Botpose", botpose.toString());
        }
    }

    public void telemetrizeLimelightPose() {
        telemetrizeLimelightPose(org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry);
    }

    public Pose getCorrectedPoseInInches() {
        Pose3D llOutput = getLimelightPose();

        if (llOutput == null) return null;

        Pose limelightPose = new Pose(llOutput.getPosition().x * Constants.Conversions.metersToInches, llOutput.getPosition().y * Constants.Conversions.metersToInches, llOutput.getOrientation().getYaw(AngleUnit.RADIANS), FTCCoordinates.INSTANCE);

        double heading = limelightPose.getHeading();
        heading -= Math.PI / 2;
        heading = heading < 0 ? heading + Math.PI * 2 : heading;

        double x = limelightPose.getY() + Constants.FieldSize.yInches / 2;
        double y = -limelightPose.getX() + Constants.FieldSize.xInches / 2;

        return new Pose(x, y, heading);
    }
}
