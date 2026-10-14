package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;

import java.util.function.DoubleSupplier;

public class TransferCommand extends CommandBase {
    private final TransferSubsystem transferSubsystem;
    private final DoubleSupplier powerSupplier;

    public TransferCommand(TransferSubsystem transferSubsystem, DoubleSupplier powerSupplier) {
        this.transferSubsystem = transferSubsystem;
        this.powerSupplier = powerSupplier;
        addRequirements(transferSubsystem);
    }

    public TransferCommand(TransferSubsystem transferSubsystem, double power) {
        this(transferSubsystem, () -> power);
    }

    public TransferCommand(TransferSubsystem transferSubsystem) {
        this(transferSubsystem, Constants.Transfer.defaultPower);
    }

    @Override
    public void execute() {
        transferSubsystem.setPower(powerSupplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        transferSubsystem.stop();
    }
}
