package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OdometrySubsystem;
import org.firstinspires.ftc.teamcode.targetingUtils.NewtonRaphsonShooterSolver;
import org.firstinspires.ftc.teamcode.targetingUtils.ShootingTargetInfo;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class PointToVirtualTargetCommand extends CommandBase {
    private final DriveSubsystem driveSubsystem;
    private final Supplier<Pose> robotPoseSupplier;
    private final Supplier<Pose> robotVelocitySupplier;
    private final Supplier<Pose> targetPoseSupplier;
    private final DoubleSupplier driverXSupplier;
    private final DoubleSupplier driverYSupplier;
    private final Telemetry telemetry;

    private double kP = 1.5; // Proportional gain for heading control
    private ShootingTargetInfo lastTargetInfo;

    public PointToVirtualTargetCommand(
            DriveSubsystem driveSubsystem,
            Supplier<Pose> robotPoseSupplier,
            Supplier<Pose> robotVelocitySupplier,
            Supplier<Pose> targetPoseSupplier,
            DoubleSupplier driverXSupplier,
            DoubleSupplier driverYSupplier,
            Telemetry telemetry
    ) {
        this.driveSubsystem = driveSubsystem;
        this.robotPoseSupplier = robotPoseSupplier;
        this.robotVelocitySupplier = robotVelocitySupplier;
        this.targetPoseSupplier = targetPoseSupplier;
        this.driverXSupplier = driverXSupplier;
        this.driverYSupplier = driverYSupplier;
        this.telemetry = telemetry;

        addRequirements(driveSubsystem);
    }

    public PointToVirtualTargetCommand(
            DriveSubsystem driveSubsystem,
            Supplier<Pose> robotPoseSupplier,
            Supplier<Pose> robotVelocitySupplier,
            Supplier<Pose> targetPoseSupplier,
            DoubleSupplier driverXSupplier,
            DoubleSupplier driverYSupplier
    ) {
        this(driveSubsystem, robotPoseSupplier, robotVelocitySupplier, targetPoseSupplier, driverXSupplier, driverYSupplier, null);
    }

    public PointToVirtualTargetCommand(
            DriveSubsystem driveSubsystem,
            OdometrySubsystem odometrySubsystem,
            Telemetry telemetry,
            DoubleSupplier driverXSupplier,
            DoubleSupplier driverYSupplier
    ) {
        this(
                driveSubsystem,
                driveSubsystem::getPose,
                odometrySubsystem::velocityVectorMetersPerSecond,
                () -> Constants.Target.position,
                driverXSupplier,
                driverYSupplier,
                telemetry
        );
    }

    public PointToVirtualTargetCommand(
            DriveSubsystem driveSubsystem,
            OdometrySubsystem odometrySubsystem,
            DoubleSupplier driverXSupplier,
            DoubleSupplier driverYSupplier
    ) {
        this(driveSubsystem, odometrySubsystem, null, driverXSupplier, driverYSupplier);
    }

    public PointToVirtualTargetCommand(
            DriveSubsystem driveSubsystem,
            OdometrySubsystem odometrySubsystem,
            Telemetry telemetry
    ) {
        this(driveSubsystem, odometrySubsystem, telemetry, () -> 0.0, () -> 0.0);
    }

    public PointToVirtualTargetCommand(
            DriveSubsystem driveSubsystem,
            OdometrySubsystem odometrySubsystem
    ) {
        this(driveSubsystem, odometrySubsystem, null, () -> 0.0, () -> 0.0);
    }

    public void setKP(double kP) {
        this.kP = kP;
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

        double targetHeading = lastTargetInfo.getTargetHeading();
        double currentHeading = robotPose.getHeading();

        // Calculate normalized heading error in [-PI, PI]
        double headingError = normalizeAngle(targetHeading - currentHeading);

        // Calculate turning velocity power (omega)
        double omega = Math.max(-1.0, Math.min(1.0, kP * headingError));

        double x = driverXSupplier.getAsDouble();
        double y = driverYSupplier.getAsDouble();

        driveSubsystem.driveFieldCentric(x, y, omega);

        if (telemetry != null) {
            telemetry.addData("Aim Target Heading (rad)", targetHeading);
            telemetry.addData("Aim Current Heading (rad)", currentHeading);
            telemetry.addData("Aim Heading Error (rad)", headingError);
            telemetry.addData("Aim Virt Target X", lastTargetInfo.getVirtualTargetPose().getX());
            telemetry.addData("Aim Virt Target Y", lastTargetInfo.getVirtualTargetPose().getY());
        }
    }

    private double normalizeAngle(double angle) {
        double normalized = angle;
        while (normalized > Math.PI) normalized -= 2 * Math.PI;
        while (normalized < -Math.PI) normalized += 2 * Math.PI;
        return normalized;
    }

    public ShootingTargetInfo getLastTargetInfo() {
        return lastTargetInfo;
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.driveFieldCentric(0, 0, 0);
    }
}
