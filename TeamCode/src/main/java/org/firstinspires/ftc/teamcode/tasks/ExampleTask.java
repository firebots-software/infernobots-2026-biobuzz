package org.firstinspires.ftc.teamcode.tasks;

import com.jumpypants.murphy.tasks.Task;
import com.jumpypants.murphy.util.RobotContext;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class ExampleTask extends Task {

    /**
     * Creates a new Task with the provided RobotContext.
     *
     * @param robotContext contains references like telemetry, gamepads, and subsystems
     */
    public ExampleTask(RobotContext robotContext) {
        super(robotContext);
    }

    @Override
    protected void initialize(RobotContext robotContext) {

    }

    @Override
    protected boolean run(RobotContext robotContext) {
        return false;
    }
}
