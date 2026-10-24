package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

import java.util.function.DoubleSupplier;

public class IntakeCommand extends CommandBase {
    private final IntakeSubsystem intakeSubsystem;
    private final DoubleSupplier powerSupplier;

    public IntakeCommand(IntakeSubsystem intakeSubsystem, DoubleSupplier powerSupplier) {
        this.intakeSubsystem = intakeSubsystem;
        this.powerSupplier = powerSupplier;
        addRequirements(intakeSubsystem);
    }

    public IntakeCommand(IntakeSubsystem intakeSubsystem, double power) {
        this(intakeSubsystem, () -> power);
    }

    public IntakeCommand(IntakeSubsystem intakeSubsystem) {
        this(intakeSubsystem, 1.0);
    }

    @Override
    public void execute() {
        intakeSubsystem.setPower(powerSupplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stop();
    }
}
