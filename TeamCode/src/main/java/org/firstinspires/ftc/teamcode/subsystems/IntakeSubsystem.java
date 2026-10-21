package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeSubsystem implements Subsystem {
    private final DcMotorEx intakeMotor;
    private double currentPower = 0.0;

    public IntakeSubsystem(HardwareMap hardwareMap, String motorName) {
        this.intakeMotor = hardwareMap.get(DcMotorEx.class, motorName);
        this.intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public IntakeSubsystem(HardwareMap hardwareMap) {
        this(hardwareMap, "intakeMotor");
    }

    public IntakeSubsystem() {
        this(org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap);
    }

    /**
     * Sets the speed/power of the intake motor.
     *
     * @param power Power level between -1.0 and 1.0.
     */
    public void setPower(double power) {
        this.currentPower = power;
        intakeMotor.setPower(power);
    }

    /**
     * Sets the speed/power of the intake motor (alias for setPower).
     *
     * @param speed Speed level between -1.0 and 1.0.
     */
    public void setSpeed(double speed) {
        setPower(speed);
    }

    /**
     * Gets the currently targeted power level of the intake motor.
     *
     * @return Current power setting.
     */
    public double getPower() {
        return currentPower;
    }

    /**
     * Gets the current speed setting of the intake motor.
     *
     * @return Current speed setting.
     */
    public double getSpeed() {
        return getPower();
    }

    /**
     * Stops the intake motor.
     */
    public void stop() {
        setPower(0.0);
    }

    /**
     * Gets the underlying DcMotorEx instance if direct access is needed.
     *
     * @return DcMotorEx instance for the intake.
     */
    public DcMotorEx getMotor() {
        return intakeMotor;
    }

    @Override
    public void periodic() {
        // Periodic updates if required
    }
}
