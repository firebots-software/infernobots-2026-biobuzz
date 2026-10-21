package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;

/**
 * Subsystem for the intake mechanism.
 */
public class IntakeSubsystem implements Subsystem {
    private final DcMotorEx intakeMotor;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, Constants.Intake.motorName);
        intakeMotor.setDirection(Constants.Intake.motorDirection);
        intakeMotor.setZeroPowerBehavior(Constants.Intake.zeroPowerBehavior);
    }

    public IntakeSubsystem() {
        this(org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap);
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
    }
}
