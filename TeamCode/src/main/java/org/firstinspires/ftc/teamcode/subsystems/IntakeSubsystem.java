package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Constants;
public class IntakeSubsystem implements Subsystem {
    private final DcMotorEx intakeMotor;

    public IntakeSubsystem() {
        intakeMotor = hardwareMap.get(DcMotorEx.class, Constants.Intake.motorName);
        intakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Spins the motor to pull game elements in.
     */
    public void intake() {
        intakeMotor.setPower(Constants.Intake.intakePower);
    }

    /**
     * Spins the motor to push game elements out.
     */
    public void outtake() {
        intakeMotor.setPower(Constants.Intake.outtakePower);
    }

    /**
     * Stops the intake motor.
     */
    public void stop() {
        intakeMotor.setPower(0);
    }

    @Override
    public void periodic() {
        // Use this for telemetry or safety logic if needed
    }
}
