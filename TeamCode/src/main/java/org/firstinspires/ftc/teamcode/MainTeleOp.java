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
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commandGroups.ShootOnTheMoveCommandGroup;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

@Configurable
@TeleOp(name="(TELE) Main", group="Main")
public class MainTeleOp extends LinearOpMode {
    public static IntakeSubsystem.TeamColor ALLIANCE_COLOR = IntakeSubsystem.TeamColor.RED;
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

        // Configure alliance color for intake subsystem and other team-specific features
//        robotContext.intakeSubsystem.setTeamColor(ALLIANCE_COLOR);

        CommandScheduler.getInstance().reset();
        CommandScheduler.getInstance().registerSubsystem(
                robotContext.driveSubsystem
//                robotContext.odometrySubsystem,
//                robotContext.shooterSubsystem,
//                robotContext.intakeSubsystem
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

//        ShootOnTheMoveCommandGroup shootOnTheMoveCommandGroup = new ShootOnTheMoveCommandGroup(
//                robotContext.shooterSubsystem,
//                robotContext.driveSubsystem,
//                robotContext.odometrySubsystem,
//                telemetry,
//                () -> Math.pow(gamepad1.left_stick_x, 3),
//                () -> Math.pow(-gamepad1.left_stick_y, 3)
//        );

        // Bind ShootOnTheMoveCommandGroup to gamepad1 right trigger
//        new Trigger(() -> gamepad1.right_trigger > 0.2)
//                .whileActiveContinuous(shootOnTheMoveCommandGroup);
//
//        // Bind IntakeCommand to gamepad1 left trigger
//        new Trigger(() -> gamepad1.left_trigger > 0.2)
//                .whileActiveContinuous(new IntakeCommand(robotContext.intakeSubsystem));

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
//            Pose odoPose = robotContext.odometrySubsystem.getWeightedPose();

            if (gamepad1.triangle) {
                robotContext.driveSubsystem.setPose(startingPose);
            }

            telemetry.addData("Alliance Color", ALLIANCE_COLOR);
//            telemetry.addData("Enemy Ball Detected", robotContext.intakeSubsystem.isEnemyBallDetected());
            telemetry.addData("x pos", currentPose.getX());
            telemetry.addData("y pos", currentPose.getY());
            telemetry.addData("heading pos", currentPose.getHeading());

//            if (odoPose != null) {
//                telemetry.addData("odo x pos", odoPose.getX());
//                telemetry.addData("odo y pos", odoPose.getY());
//                telemetry.addData("odo heading pos", odoPose.getHeading());
//            }

            telemetry.update();
        }
    }
}
