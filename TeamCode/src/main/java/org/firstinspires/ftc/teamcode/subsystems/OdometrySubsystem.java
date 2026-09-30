package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.arcrobotics.ftclib.command.Subsystem;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.LimelightLocalizer;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;

public class OdometrySubsystem implements Subsystem {
    LimelightLocalizer limelight = new LimelightLocalizer(hardwareMap.get(Limelight3A.class, "cam"));
    GoBildaPinpointDriver odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

    private double limeLightConfidence = 0f;

    public OdometrySubsystem() {
        GoBildaPinpointDriver odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        // 2. Set encoder resolution (e.g., ticks per mm for goBILDA pods)
        // GoBildaOdometryPods.goBILDA_4_BAR_POD = 19.20531005221992
        // GoBildaOdometryPods.goBILDA_SWINGARM_POD = 13.26291192
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        // 3. Set the physical offsets of your pods relative to the robot's center (in mm)
        // parOffset: distance from center to parallel wheel (positive = left)
        // perpOffset: distance from center to perpendicular wheel (positive = forward/back)
        odo.setOffsets(Constants.OdometryWheels.parOffsetMM, Constants.OdometryWheels.perpOffsetMM, DistanceUnit.MM);

        // 4. Match your structural encoder wiring directions
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);

        // 5. Optionally reset the position at autonomous start
        // odo.resetPosAndIMU();
    }

    public Pose getWeightedPose() {
        return new Pose(limelight.getLimelightPose().getPosition().x * limeLightConfidence + odo.getPosX(DistanceUnit.METER) * (1-limeLightConfidence),
                        limelight.getLimelightPose().getPosition().y * limeLightConfidence + odo.getPosX(DistanceUnit.METER) * (1-limeLightConfidence),
                   limelight.getLimelightPose().getOrientation().getYaw() * limeLightConfidence + odo.getHeading(AngleUnit.RADIANS) * (1-limeLightConfidence));
    }

    public Pose velocityVectorMetersPerSecond() {
        return new Pose(odo.getVelX(DistanceUnit.METER), odo.getVelY(DistanceUnit.METER), odo.getHeadingVelocity(AngleUnit.RADIANS.getUnnormalized()));
    }

    public void updateOdo() {
        odo.update();
    }
}
