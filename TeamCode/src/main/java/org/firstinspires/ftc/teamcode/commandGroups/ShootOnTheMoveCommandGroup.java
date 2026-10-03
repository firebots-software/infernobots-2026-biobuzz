package org.firstinspires.ftc.teamcode.commandGroups;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commands.PointToVirtualTargetCommand;
import org.firstinspires.ftc.teamcode.commands.SetShootSpeedCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OdometrySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class ShootOnTheMoveCommandGroup extends ParallelCommandGroup {

    public ShootOnTheMoveCommandGroup(
            ShooterSubsystem shooterSubsystem,
            DriveSubsystem driveSubsystem,
            Supplier<Pose> robotPoseSupplier,
            Supplier<Pose> robotVelocitySupplier,
            Supplier<Pose> targetPoseSupplier,
            DoubleSupplier driverXSupplier,
            DoubleSupplier driverYSupplier,
            Telemetry telemetry
    ) {
        addCommands(
                new SetShootSpeedCommand(
                        shooterSubsystem,
                        robotPoseSupplier,
                        robotVelocitySupplier,
                        targetPoseSupplier,
                        telemetry
                ),
                new PointToVirtualTargetCommand(
                        driveSubsystem,
                        robotPoseSupplier,
                        robotVelocitySupplier,
                        targetPoseSupplier,
                        driverXSupplier,
                        driverYSupplier,
                        telemetry
                )
        );
    }

    public ShootOnTheMoveCommandGroup(
            ShooterSubsystem shooterSubsystem,
            DriveSubsystem driveSubsystem,
            Supplier<Pose> robotPoseSupplier,
            Supplier<Pose> robotVelocitySupplier,
            Supplier<Pose> targetPoseSupplier,
            DoubleSupplier driverXSupplier,
            DoubleSupplier driverYSupplier
    ) {
        this(shooterSubsystem, driveSubsystem, robotPoseSupplier, robotVelocitySupplier, targetPoseSupplier, driverXSupplier, driverYSupplier, null);
    }

    public ShootOnTheMoveCommandGroup(
            ShooterSubsystem shooterSubsystem,
            DriveSubsystem driveSubsystem,
            OdometrySubsystem odometrySubsystem,
            Telemetry telemetry,
            DoubleSupplier driverXSupplier,
            DoubleSupplier driverYSupplier
    ) {
        this(
                shooterSubsystem,
                driveSubsystem,
                driveSubsystem::getPose,
                odometrySubsystem::velocityVectorMetersPerSecond,
                () -> Constants.Target.position,
                driverXSupplier,
                driverYSupplier,
                telemetry
        );
    }

    public ShootOnTheMoveCommandGroup(
            ShooterSubsystem shooterSubsystem,
            DriveSubsystem driveSubsystem,
            OdometrySubsystem odometrySubsystem,
            DoubleSupplier driverXSupplier,
            DoubleSupplier driverYSupplier
    ) {
        this(shooterSubsystem, driveSubsystem, odometrySubsystem, null, driverXSupplier, driverYSupplier);
    }

    public ShootOnTheMoveCommandGroup(
            ShooterSubsystem shooterSubsystem,
            DriveSubsystem driveSubsystem,
            OdometrySubsystem odometrySubsystem,
            Telemetry telemetry
    ) {
        this(shooterSubsystem, driveSubsystem, odometrySubsystem, telemetry, () -> 0.0, () -> 0.0);
    }

    public ShootOnTheMoveCommandGroup(
            ShooterSubsystem shooterSubsystem,
            DriveSubsystem driveSubsystem,
            OdometrySubsystem odometrySubsystem
    ) {
        this(shooterSubsystem, driveSubsystem, odometrySubsystem, null, () -> 0.0, () -> 0.0);
    }
}
