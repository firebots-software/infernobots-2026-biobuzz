package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

import java.util.function.DoubleSupplier;

public class IntakeCommand extends CommandBase {
    private final IntakeSubsystem intakeSubsystem;
    private final DoubleSupplier powerSupplier;

    /**
     * Creates an IntakeCommand that runs the intake subsystem at a dynamic power level.
     *
     * @param intakeSubsystem The intake subsystem to control.
     * @param powerSupplier   Supplier providing the power level (-1.0 to 1.0).
     */
    public IntakeCommand(IntakeSubsystem intakeSubsystem, DoubleSupplier powerSupplier) {
        this.intakeSubsystem = intakeSubsystem;
        this.powerSupplier = powerSupplier;
        addRequirements(intakeSubsystem);
    }

    /**
     * Creates an IntakeCommand that runs the intake subsystem at a constant power level.
     *
     * @param intakeSubsystem The intake subsystem to control.
     * @param power           The fixed power level (-1.0 to 1.0).
     */
    public IntakeCommand(IntakeSubsystem intakeSubsystem, double power) {
        this(intakeSubsystem, () -> power);
    }

    @Override
    public void initialize() {
        intakeSubsystem.setPower(powerSupplier.getAsDouble());
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
