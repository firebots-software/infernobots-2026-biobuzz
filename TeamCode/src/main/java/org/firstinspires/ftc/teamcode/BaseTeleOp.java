package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.jumpypants.murphy.states.StateMachine;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commandGroups.ShootOnTheMoveCommandGroup;

@Configurable
public class BaseTeleOp extends LinearOpMode {
    StateMachine stateMachine;

    @Override
    public void runOpMode() {
        MyRobot robotContext = new MyRobot(
                hardwareMap,
                telemetry,
                gamepad1,
                gamepad2
        );
        TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        CommandScheduler.getInstance().reset();
        CommandScheduler.getInstance().registerSubsystem(
                robotContext.driveSubsystem,
                robotContext.odometrySubsystem,
                robotContext.shooterSubsystem
        );

        // Set default command for driveSubsystem (runs whenever no other command requires driveSubsystem)
        robotContext.driveSubsystem.setDefaultCommand(
                new RunCommand(
                        () -> robotContext.driveSubsystem.driveFieldCentric(
                                Math.pow(gamepad1.left_stick_x, 3),
                                Math.pow(-gamepad1.left_stick_y, 3),
                                Math.pow(-gamepad1.right_stick_x, 3)
                        ),
                        robotContext.driveSubsystem
                )
        );

        ShootOnTheMoveCommandGroup shootOnTheMoveCommandGroup = new ShootOnTheMoveCommandGroup(
                robotContext.shooterSubsystem,
                robotContext.driveSubsystem,
                robotContext.odometrySubsystem,
                telemetry,
                () -> Math.pow(gamepad1.left_stick_x, 3),
                () -> Math.pow(-gamepad1.left_stick_y, 3)
        );

        // Bind ShootOnTheMoveCommandGroup to gamepad1 right trigger
        new Trigger(() -> gamepad1.right_trigger > 0.2)
                .whileActiveContinuous(shootOnTheMoveCommandGroup);

        Pose startingPose = new Pose(0, 0, Math.PI / 2);
        robotContext.driveSubsystem.setStartingPose(startingPose);

        waitForStart();

        robotContext.driveSubsystem.switchToTeleOp();

        while (opModeIsActive()) {
            CommandScheduler.getInstance().run();

            if (stateMachine != null) {
                stateMachine.step();
            }

            Pose currentPose = robotContext.driveSubsystem.getPose();
            Pose odoPose = robotContext.odometrySubsystem.getWeightedPose();

            if (gamepad1.triangle) {
                robotContext.driveSubsystem.setPose(startingPose);
            }

            telemetry.addData("x pos", currentPose.getX());
            telemetry.addData("y pos", currentPose.getY());
            telemetry.addData("heading pos", currentPose.getHeading());

            if (odoPose != null) {
                telemetry.addData("odo x pos", odoPose.getX());
                telemetry.addData("odo y pos", odoPose.getY());
                telemetry.addData("odo heading pos", odoPose.getHeading());
            }

            telemetry.update();
        }
    }
}