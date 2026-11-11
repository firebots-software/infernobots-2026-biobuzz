package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;

public class TransferCommand extends CommandBase {
    private final TransferSubsystem transferSubsystem;

    public TransferCommand(TransferSubsystem transferSubsystem) {
        this.transferSubsystem = transferSubsystem;
        addRequirements(transferSubsystem);
    }

    @Override
    public void initialize() {
        transferSubsystem.transfer();
    }

    @Override
    public void end(boolean interrupted) {
        transferSubsystem.stop();
    }
}
