package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;

public class TransferSubsystem implements Subsystem {
    private final DcMotorEx transferMotor;

    public TransferSubsystem(HardwareMap hardwareMap) {
        transferMotor = hardwareMap.get(DcMotorEx.class, Constants.Transfer.motorName);
        transferMotor.setDirection(Constants.Transfer.motorDirection);
        transferMotor.setZeroPowerBehavior(Constants.Transfer.zeroPowerBehavior);
    }

    public TransferSubsystem() {
        this(org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap);
    }

    /**
     * Spins the motor to transfer balls forward to the turret.
     */
    public void transfer() {
        transferMotor.setPower(Constants.Transfer.transferPower);
    }

    /**
     * Spins the motor in reverse to clear jams.
     */
    public void reverse() {
        transferMotor.setPower(Constants.Transfer.reversePower);
    }

    /**
     * Stops the transfer motor.
     */
    public void stop() {
        transferMotor.setPower(0);
    }

    @Override
    public void periodic() {
        // Runs once per scheduler loop
    }
}
