package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TransferSubsystem implements Subsystem {
    private final DcMotorEx transferMotor;
    private double currentPower = 0.0;

    public TransferSubsystem(HardwareMap hardwareMap) {
        this.transferMotor = hardwareMap.get(DcMotorEx.class, "transferMotor");
        this.transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.transferMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public TransferSubsystem() {
        this(org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap);
    }

    public void setPower(double power) {
        this.currentPower = power;
        transferMotor.setPower(power);
    }

    public double getPower() {
        return currentPower;
    }

    public void stop() {
        currentPower = 0.0;
        transferMotor.setPower(0.0);
    }

    @Override
    public void periodic() {
        // Periodic updates if required
    }
}
