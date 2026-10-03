package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OdometrySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.targetingUtils.NewtonRaphsonShooterSolver;
import org.firstinspires.ftc.teamcode.targetingUtils.ShootingTargetInfo;

import java.util.function.Supplier;

public class SetShootSpeedCommand extends CommandBase {
    private final ShooterSubsystem shooterSubsystem;
    private final Supplier<Pose> robotPoseSupplier;
    private final Supplier<Pose> robotVelocitySupplier;
    private final Supplier<Pose> targetPoseSupplier;
    private final Telemetry telemetry;

    private ShootingTargetInfo lastTargetInfo;

    public SetShootSpeedCommand(
            ShooterSubsystem shooterSubsystem,
            Supplier<Pose> robotPoseSupplier,
            Supplier<Pose> robotVelocitySupplier,
            Supplier<Pose> targetPoseSupplier,
            Telemetry telemetry
    ) {
        this.shooterSubsystem = shooterSubsystem;
        this.robotPoseSupplier = robotPoseSupplier;
        this.robotVelocitySupplier = robotVelocitySupplier;
        this.targetPoseSupplier = targetPoseSupplier;
        this.telemetry = telemetry;

        addRequirements(shooterSubsystem);
    }

    public SetShootSpeedCommand(
            ShooterSubsystem shooterSubsystem,
            Supplier<Pose> robotPoseSupplier,
            Supplier<Pose> robotVelocitySupplier,
            Supplier<Pose> targetPoseSupplier
    ) {
        this(shooterSubsystem, robotPoseSupplier, robotVelocitySupplier, targetPoseSupplier, null);
    }

    public SetShootSpeedCommand(
            ShooterSubsystem shooterSubsystem,
            DriveSubsystem driveSubsystem,
            OdometrySubsystem odometrySubsystem,
            Telemetry telemetry
    ) {
        this(
                shooterSubsystem,
                driveSubsystem::getPose,
                odometrySubsystem::velocityVectorMetersPerSecond,
                () -> Constants.Target.position,
                telemetry
        );
    }

    public SetShootSpeedCommand(
            ShooterSubsystem shooterSubsystem,
            DriveSubsystem driveSubsystem,
            OdometrySubsystem odometrySubsystem
    ) {
        this(shooterSubsystem, driveSubsystem, odometrySubsystem, null);
    }

    @Override
    public void execute() {
        Pose robotPose = robotPoseSupplier.get();
        Pose robotVelocity = robotVelocitySupplier.get();
        Pose targetPose = targetPoseSupplier.get();

        if (robotPose == null) robotPose = new Pose(0, 0, 0);
        if (robotVelocity == null) robotVelocity = new Pose(0, 0, 0);
        if (targetPose == null) targetPose = new Pose(0, 0, 0);

        lastTargetInfo = NewtonRaphsonShooterSolver.calculateTargeting(
                robotPose,
                robotVelocity,
                targetPose,
                Constants.Shooter.distanceToSpeed,
                Constants.Shooter.distanceToTimeOfFlight
        );

        double targetSpeed = lastTargetInfo.getTargetShooterSpeed();
        shooterSubsystem.setVelocity(targetSpeed);

        if (telemetry != null) {
            telemetry.addData("Shooter Target Speed", targetSpeed);
            telemetry.addData("Shooter Actual Speed", shooterSubsystem.getVelocity());
            telemetry.addData("Shooter Virt Distance", lastTargetInfo.getVirtualDistance());
            telemetry.addData("Shooter Time of Flight", lastTargetInfo.getTimeOfFlight());
        }
    }

    public ShootingTargetInfo getLastTargetInfo() {
        return lastTargetInfo;
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.stop();
    }
}
