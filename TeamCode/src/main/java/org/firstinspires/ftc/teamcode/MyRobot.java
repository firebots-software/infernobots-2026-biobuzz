package org.firstinspires.ftc.teamcode;

import com.jumpypants.murphy.util.RobotContext;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OdometrySubsystem;

/**
 * MyRobot class that extends RobotContext to include robot-specific subsystems.
 */
public class MyRobot extends RobotContext {
    public static Follower follower = null;

    public final HardwareMap HARDWARE_MAP;
    public final DriveSubsystem driveSubsystem;
    public final OdometrySubsystem odometrySubsystem;

    /**
     * Creates a new RobotContext with the specified hardware map, telemetry and gamepad references.
     * All parameters are required and cannot be null.
     *
     * @param hardwareMap the hardware map for accessing hardware components
     * @param telemetry   the telemetry instance for driver station communication
     * @param gamepad1    the primary gamepad controller
     * @param gamepad2    the secondary gamepad controller
     */
    public MyRobot(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2) {
        super(telemetry, gamepad1, gamepad2);
        this.HARDWARE_MAP = hardwareMap;
        this.driveSubsystem = new DriveSubsystem(hardwareMap);
        this.odometrySubsystem = new OdometrySubsystem(hardwareMap);
        follower = driveSubsystem.getFollower();
    }
}