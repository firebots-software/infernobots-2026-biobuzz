package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.LimelightLocalizer;

public class OdometrySubsystem implements Subsystem {
    private final LimelightLocalizer limelight;
    private final GoBildaPinpointDriver odo;

    public OdometrySubsystem(HardwareMap hardwareMap) {
        this.limelight = new LimelightLocalizer(hardwareMap.get(Limelight3A.class, "cam"));
        this.limelight.start();
        this.odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        // Set encoder resolution (e.g., ticks per mm for goBILDA pods)
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        // Set physical offsets of pods relative to center (in mm)
        odo.setOffsets(Constants.OdometryWheels.parOffsetMM, Constants.OdometryWheels.perpOffsetMM, DistanceUnit.MM);

        // Match structural encoder wiring directions
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
    }

    public OdometrySubsystem() {
        this(org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap);
    }

    public void setPose(Pose pose) {
        if (pose != null) {
            odo.setPosition(new Pose2D(DistanceUnit.INCH, pose.getX(), pose.getY(), AngleUnit.RADIANS, pose.getHeading()));
        }
    }

    public void resetPose() {
        setPose(new Pose(0, 0, 0));
    }

    public Pose getPose() {
        return new Pose(
                odo.getPosX(DistanceUnit.INCH),
                odo.getPosY(DistanceUnit.INCH),
                odo.getHeading(AngleUnit.RADIANS)
        );
    }

    public Pose getWeightedPose() {
        return getPose();
    }

    public Pose velocityVectorMetersPerSecond() {
        return new Pose(odo.getVelX(DistanceUnit.METER), odo.getVelY(DistanceUnit.METER), odo.getHeadingVelocity(AngleUnit.RADIANS.getUnnormalized()));
    }

    public void updateOdo() {
        odo.update();
    }

    @Override
    public void periodic() {
        updateOdo();
        Pose llPose = limelight.getCorrectedPoseInInches();
        if (llPose != null) {
            setPose(llPose);
        }
    }
}
