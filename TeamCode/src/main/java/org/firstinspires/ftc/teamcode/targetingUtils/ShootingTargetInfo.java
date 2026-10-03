package org.firstinspires.ftc.teamcode.targetingUtils;

import com.pedropathing.geometry.Pose;

public class ShootingTargetInfo {
    private final double targetHeading;
    private final double targetShooterSpeed;
    private final double virtualDistance;
    private final double timeOfFlight;
    private final Pose virtualTargetPose;

    public ShootingTargetInfo(
            double targetHeading,
            double targetShooterSpeed,
            double virtualDistance,
            double timeOfFlight,
            Pose virtualTargetPose
    ) {
        this.targetHeading = targetHeading;
        this.targetShooterSpeed = targetShooterSpeed;
        this.virtualDistance = virtualDistance;
        this.timeOfFlight = timeOfFlight;
        this.virtualTargetPose = virtualTargetPose;
    }

    public double getTargetHeading() {
        return targetHeading;
    }

    public double getTargetShooterSpeed() {
        return targetShooterSpeed;
    }

    public double getVirtualDistance() {
        return virtualDistance;
    }

    public double getTimeOfFlight() {
        return timeOfFlight;
    }

    public Pose getVirtualTargetPose() {
        return virtualTargetPose;
    }
}
