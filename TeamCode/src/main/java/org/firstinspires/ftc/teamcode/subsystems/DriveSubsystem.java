package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.arcrobotics.ftclib.command.Subsystem;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.LimelightLocalizer;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class DriveSubsystem implements Subsystem {
    Follower follower;

    public DriveSubsystem() {
        follower = Constants.createFollower(hardwareMap);
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
