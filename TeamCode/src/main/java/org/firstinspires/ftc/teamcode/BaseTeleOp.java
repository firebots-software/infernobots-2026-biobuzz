package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.jumpypants.murphy.states.StateMachine;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

@Configurable
public class BaseTeleOp extends LinearOpMode {
    StateMachine stateMachine;

    DriveSubsystem driveSubsystem = new DriveSubsystem();

    @Override
    public void runOpMode() {
        MyRobot robotContext = new MyRobot(
                hardwareMap,
                telemetry,
                gamepad1,
                gamepad2
        );
        TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        waitForStart();

        Pose startingPose = new Pose(0, 0, Math.PI / 2);
        driveSubsystem.setStartingPose(startingPose);

        driveSubsystem.switchToTeleOp();

        while (opModeIsActive()){
            CommandScheduler.getInstance().run();

            stateMachine.step();

            Pose currentPose = driveSubsystem.getPose();

            if (gamepad1.triangle) {
                driveSubsystem.setPose(startingPose);
            }

            driveSubsystem.driveFieldCentric(
                    Math.pow(gamepad1.left_stick_x, 3),
                    Math.pow(-gamepad1.left_stick_y, 3),
                    Math.pow(-gamepad1.right_stick_x, 3)
            );

            telemetry.addData("x pos", currentPose.getX());
            telemetry.addData("y pos", currentPose.getY());
            telemetry.addData("heading pos", currentPose.getHeading());

            telemetry.update();
        }
    }
}