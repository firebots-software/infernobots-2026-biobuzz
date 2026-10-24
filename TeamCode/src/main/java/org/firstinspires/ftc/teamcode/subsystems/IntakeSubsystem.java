package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

public class IntakeSubsystem implements Subsystem {
    public enum TeamColor {
        RED, BLUE
    }

    private final DcMotorEx intakeMotor;
    private NormalizedColorSensor colorSensor;

    private TeamColor myTeamColor = TeamColor.RED;
    private double targetPower = 0.0;
    private static final double PROXIMITY_ALPHA_THRESHOLD = 0.05; // Light threshold for detecting object presence
    private static final double EJECT_POWER = -1.0; // Reverse power to eject enemy ball

    public IntakeSubsystem(HardwareMap hardwareMap, String motorName, String colorSensorName) {
        this.intakeMotor = hardwareMap.get(DcMotorEx.class, motorName);
        this.intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        try {
            this.colorSensor = hardwareMap.get(NormalizedColorSensor.class, colorSensorName);
        } catch (Exception e) {
            this.colorSensor = null; // Fallback if color sensor hardware is not present/configured
        }
    }

    public IntakeSubsystem(HardwareMap hardwareMap) {
        this(hardwareMap, "intakeMotor", "colorSensor");
    }

    public IntakeSubsystem() {
        this(org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap);
    }

    public void setTeamColor(TeamColor teamColor) {
        this.myTeamColor = teamColor;
    }

    public TeamColor getTeamColor() {
        return myTeamColor;
    }

    /**
     * Detects if an enemy ball (opposing alliance color) is present at the color sensor.
     *
     * @return true if an enemy ball is detected, false otherwise.
     */
    public boolean isEnemyBallDetected() {
        if (colorSensor == null) return false;

        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        // Check if an object/ball is close enough to the sensor
        if (colors.alpha < PROXIMITY_ALPHA_THRESHOLD) {
            return false;
        }

        boolean isRedBall = colors.red > (colors.blue * 1.4);
        boolean isBlueBall = colors.blue > (colors.red * 1.4);

        if (myTeamColor == TeamColor.RED) {
            return isBlueBall; // Enemy ball if team is RED and sensor sees BLUE
        } else {
            return isRedBall;  // Enemy ball if team is BLUE and sensor sees RED
        }
    }

    /**
     * Sets the speed/power of the intake motor.
     * If an enemy ball is detected while intaking, the power is automatically overridden to reverse.
     *
     * @param power Power level between -1.0 and 1.0.
     */
    public void setPower(double power) {
        this.targetPower = power;
        updateMotorPower();
    }

    public void setSpeed(double speed) {
        setPower(speed);
    }

    public double getPower() {
        return targetPower;
    }

    public double getSpeed() {
        return getPower();
    }

    public void stop() {
        setPower(0.0);
    }

    public DcMotorEx getMotor() {
        return intakeMotor;
    }

    public NormalizedColorSensor getColorSensor() {
        return colorSensor;
    }

    private void updateMotorPower() {
        // Anti-Enemy Reversing Override: If trying to intake forward (targetPower > 0)
        // AND an enemy ball is detected, force motor into REVERSE (-1.0) to eject the ball!
        if (targetPower > 0 && isEnemyBallDetected()) {
            intakeMotor.setPower(EJECT_POWER);
        } else {
            intakeMotor.setPower(targetPower);
        }
    }

    @Override
    public void periodic() {
        updateMotorPower();
    }
}
