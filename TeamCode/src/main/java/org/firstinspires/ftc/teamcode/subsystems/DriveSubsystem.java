package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class DriveSubsystem implements Subsystem {
    private final Follower follower;

    public DriveSubsystem(HardwareMap hardwareMap) {
        follower = Constants.createFollower(hardwareMap);
    }

    public DriveSubsystem() {
        this(org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap);
    }

    public Follower getFollower() {
        return follower;
    }

    public void driveFieldCentric(double x, double y, double omega) {
        follower.setTeleOpDrive(x, y, omega, false);
    }

    public void driveRobotCentric(double x, double y, double omega) {
        follower.setTeleOpDrive(x, y, omega, true);
    }

    public Pose getPose() {
        return follower.getPose();
    }

    public void setPose(Pose pose) {
        follower.setPose(pose);
        follower.update();
    }

    public void setStartingPose(Pose pose) {
        follower.setStartingPose(pose);
        follower.update();
    }

    public void switchToTeleOp() {
        follower.startTeleopDrive();
    }

    @Override
    public void periodic() {
        follower.update();
    }
}
