package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.bylazar.configurables.annotations.Configurable;
import com.jumpypants.murphy.tasks.SequentialTask;
import com.jumpypants.murphy.tasks.Task;
import com.jumpypants.murphy.tasks.WaitTask;
import com.jumpypants.murphy.util.RobotContext;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Modular FTC Autonomous Framework for Biobuzz.
 * 
 * Features:
 * 1. Pose/Position Placeholders: Track current estimated robot position (X, Y, Heading).
 * 2. Path Storage: HashMap storing preset waypoints mapped to alliance start positions ("RED_NEAR", "BLUE_FAR", etc.).
 * 3. Modular Coordinate Methods: Helper method driveToPose for target vector coordinates.
 * 4. State Machine Integration: Dynamic execution of paths based on selected alliance and location.
 */
@Configurable
@Autonomous(name = "(AUTON) Biobuzz Main Auton", group = "Main")
public class Auton extends LinearOpMode {

    // =========================================================================
    // 1. ALLIANCE & STARTING LOCATION SELECTION
    // =========================================================================
    public enum Alliance {
        RED, BLUE
    }

    public enum StartLocation {
        NEAR, FAR
    }

    public static Alliance alliance = Alliance.BLUE;
    public static StartLocation startLocation = StartLocation.FAR;

    // Static pose storage for hand-off to TeleOp
    public static Pose savedAutonEndPose = new Pose(0, 0, 0);
    public static Alliance savedAlliance = Alliance.BLUE;

    // =========================================================================
    // 2. POSITION & PATH STORAGE PLACEHOLDERS
    // =========================================================================
    // Lightweight Pose tracking variables (X, Y in inches, Heading in radians)
    private double currentX = 0.0;
    private double currentY = 0.0;
    private double currentHeadingRad = 0.0;

    // Default landmark poses (in field-centric inches and radians)
    private final Pose START_POSE_FAR = new Pose(57, 8.5, 0.5 * Math.PI);
    private final Pose START_POSE_NEAR = new Pose(12, 8.5, 0.5 * Math.PI);

    // Structural Collection storing preset movement paths mapped to starting keys ("RED_FAR", "BLUE_NEAR", etc.)
    private final Map<String, List<Pose>> pathStorage = new HashMap<>();
    private final List<Path> generatedPedroPaths = new ArrayList<>();

    // =========================================================================
    // 3. STATE MACHINE DEFINITION
    // =========================================================================
    public enum AutonState {
        INIT,
        START_PATH,
        EXECUTING_PATH,
        MECHANISM_ACTION,
        PARK,
        COMPLETED
    }

    private AutonState currentState = AutonState.INIT;

    // Robot & PedroPathing References
    private MyRobot robotContext;
    private Follower follower;

    @Override
    public void runOpMode() {
        // Initialize MyRobot Context
        robotContext = new MyRobot(
                hardwareMap,
                telemetry,
                gamepad1,
                gamepad2
        );

        follower = MyRobot.follower;

        // Populate preset path waypoints in pathStorage HashMap
        loadAutonPaths();

        // ---------------------------------------------------------------------
        // INIT LOOP: Gamepad Alliance & Starting Location Selection
        // ---------------------------------------------------------------------
        while (opModeInInit()) {
            if (gamepad2.left_bumper) {
                alliance = Alliance.BLUE;
            } else if (gamepad2.right_bumper) {
                alliance = Alliance.RED;
            }

            if (gamepad2.dpad_up) {
                startLocation = StartLocation.NEAR;
            } else if (gamepad2.dpad_down) {
                startLocation = StartLocation.FAR;
            }

            telemetry.addLine("=== BIOBUZZ AUTONOMOUS FRAMEWORK ===");
            telemetry.addData("Selected Alliance", alliance.name());
            telemetry.addData("Selected Start Location", startLocation.name());
            telemetry.addData("Active Path Key", getAlliancePathKey());
            telemetry.update();
        }

        // Configure starting pose based on alliance choice
        Pose baseStart = (startLocation == StartLocation.FAR) ? START_POSE_FAR : START_POSE_NEAR;
        Pose startingPose = mirrorForAlliance(baseStart.getX(), baseStart.getY(), baseStart.getHeading());

        if (follower != null) {
            follower.setStartingPose(startingPose);
        }

        updateCurrentPosition(startingPose.getX(), startingPose.getY(), startingPose.getHeading());

        // Build PedroPathing path objects from pathStorage waypoints
        buildPedroPathsFromStorage();

        // Build Murphy Task Sequence
        Task mainTask = buildMainTask(robotContext, follower, generatedPedroPaths);

        currentState = AutonState.START_PATH;

        waitForStart();

        // ---------------------------------------------------------------------
        // MAIN AUTONOMOUS EXECUTION LOOP
        // ---------------------------------------------------------------------
        while (opModeIsActive() && !isStopRequested()) {
            if (follower != null) {
                follower.update();
                Pose p = follower.getPose();
                if (p != null) {
                    updateCurrentPosition(p.getX(), p.getY(), p.getHeading());
                }
            }

            // Save ending pose continuously for seamless TeleOp hand-off
            savedAutonEndPose = new Pose(currentX, currentY, currentHeadingRad);
            savedAlliance = alliance;

            // Execute Murphy Task Sequence step
            if (mainTask != null) {
                mainTask.step();
            }

            // Execute State Machine
            runStateMachine();

            // Telemetry Status
            telemetry.addData("Auton State", currentState);
            telemetry.addData("Robot Estimated X", currentX);
            telemetry.addData("Robot Estimated Y", currentY);
            telemetry.addData("Robot Heading (deg)", Math.toDegrees(currentHeadingRad));
            telemetry.update();
        }
    }

    // =========================================================================
    // 4. PATH STORAGE & MODULAR VECTOR / COORDINATE METHODS
    // =========================================================================

    /**
     * Loads preset path coordinates inside the pathStorage HashMap.
     */
    private void loadAutonPaths() {
        pathStorage.clear();

        // Path for "BLUE_FAR"
        List<Pose> blueFarWaypoints = new ArrayList<>();
        blueFarWaypoints.add(new Pose(61, 22, 0.5 * Math.PI));
        blueFarWaypoints.add(new Pose(36.3, 11, 0.5 * Math.PI));
        pathStorage.put("BLUE_FAR", blueFarWaypoints);

        // Path for "BLUE_NEAR"
        List<Pose> blueNearWaypoints = new ArrayList<>();
        blueNearWaypoints.add(new Pose(24, 36, 0.5 * Math.PI));
        blueNearWaypoints.add(new Pose(36, 48, 0.0));
        pathStorage.put("BLUE_NEAR", blueNearWaypoints);

        // Path for "RED_FAR"
        List<Pose> redFarWaypoints = new ArrayList<>();
        redFarWaypoints.add(mirrorForAlliance(61, 22, 0.5 * Math.PI));
        redFarWaypoints.add(mirrorForAlliance(36.3, 11, 0.5 * Math.PI));
        pathStorage.put("RED_FAR", redFarWaypoints);

        // Path for "RED_NEAR"
        List<Pose> redNearWaypoints = new ArrayList<>();
        redNearWaypoints.add(mirrorForAlliance(24, 36, 0.5 * Math.PI));
        redNearWaypoints.add(mirrorForAlliance(36, 48, 0.0));
        pathStorage.put("RED_NEAR", redNearWaypoints);
    }

    /**
     * Helper method returning the current path key string ("BLUE_FAR", "RED_NEAR", etc.).
     */
    public String getAlliancePathKey() {
        return alliance.name() + "_" + startLocation.name();
    }

    /**
     * Updates internal position tracking variables.
     */
    public void updateCurrentPosition(double x, double y, double headingRad) {
        this.currentX = x;
        this.currentY = y;
        this.currentHeadingRad = headingRad;
    }

    /**
     * Placeholder method to drive the robot directly to a target coordinate vector.
     * 
     * TODO: Integrate with RoadRunner, FTC-Lib trajectory followers, or internal odometry PID localizer.
     */
    public void driveToPose(double targetX, double targetY, double targetHeading) {
        // TODO: Calculate distance delta: dx = targetX - currentX, dy = targetY - currentY
        // TODO: Pass targetPose to follower / drivetrain PID controller
        if (follower != null) {
            Pose target = new Pose(targetX, targetY, targetHeading);
            Path p = new Path(new BezierLine(new Pose(currentX, currentY, currentHeadingRad), target));
            follower.followPath(p);
        }
    }

    /**
     * Mirrors coordinates for RED alliance (X -> 144 - X, Heading -> PI - Heading).
     */
    public Pose mirrorForAlliance(double x, double y, double heading) {
        if (alliance == Alliance.BLUE) {
            return new Pose(x, y, heading);
        } else {
            return new Pose(144.0 - x, y, Math.PI - heading);
        }
    }

    /**
     * Converts waypoints stored in pathStorage for the active path key into PedroPathing Path objects.
     */
    private void buildPedroPathsFromStorage() {
        generatedPedroPaths.clear();
        String activeKey = getAlliancePathKey();
        List<Pose> waypoints = pathStorage.get(activeKey);

        if (waypoints == null || waypoints.isEmpty()) return;

        Pose lastPose = (startLocation == StartLocation.FAR) ?
                mirrorForAlliance(START_POSE_FAR.getX(), START_POSE_FAR.getY(), START_POSE_FAR.getHeading()) :
                mirrorForAlliance(START_POSE_NEAR.getX(), START_POSE_NEAR.getY(), START_POSE_NEAR.getHeading());

        for (Pose targetPose : waypoints) {
            Path path = new Path(new BezierLine(lastPose, targetPose));
            path.setLinearHeadingInterpolation(lastPose.getHeading(), targetPose.getHeading());
            generatedPedroPaths.add(path);
            lastPose = targetPose;
        }
    }

    // =========================================================================
    // 5. MURPHY TASK SEQUENCE BUILDER & STATE MACHINE INTEGRATION
    // =========================================================================

    @NonNull
    private Task buildMainTask(MyRobot robotContext, Follower follower, List<Path> pedroPaths) {
        if (pedroPaths.isEmpty()) {
            return new WaitTask(robotContext, 0.1);
        }

        // Sequences: Drive Path 0 -> Wait 1 sec -> Drive Path 1 (if present)
        if (pedroPaths.size() >= 2) {
            return new SequentialTask(robotContext,
                    new FollowPathTask(robotContext, follower, pedroPaths.get(0)),
                    new WaitTask(robotContext, 1.0),
                    new FollowPathTask(robotContext, follower, pedroPaths.get(1))
            );
        } else {
            return new SequentialTask(robotContext,
                    new FollowPathTask(robotContext, follower, pedroPaths.get(0))
            );
        }
    }

    /**
     * Switch-case state machine dynamically handling high-level autonomous phases.
     */
    private void runStateMachine() {
        switch (currentState) {
            case INIT:
                // Waiting for start
                break;

            case START_PATH:
                currentState = AutonState.EXECUTING_PATH;
                break;

            case EXECUTING_PATH:
                if (follower != null && !follower.isBusy()) {
                    currentState = AutonState.MECHANISM_ACTION;
                }
                break;

            case MECHANISM_ACTION:
                // TODO: Perform mechanism actions (Shooter, Intake, Transfer)
                currentState = AutonState.PARK;
                break;

            case PARK:
                // Finished
                currentState = AutonState.COMPLETED;
                break;

            case COMPLETED:
                break;
        }
    }

    // =========================================================================
    // 6. MURPHY FOLLOW PATH TASK
    // =========================================================================
    private static class FollowPathTask extends Task {
        private final Follower follower;
        private final Path path;

        public FollowPathTask(RobotContext robotContext, Follower follower, Path path) {
            super(robotContext);
            this.follower = follower;
            this.path = path;
        }

        @Override
        protected void initialize(RobotContext robotContext) {
            if (follower != null && path != null) {
                follower.followPath(path);
            }
        }

        @Override
        protected boolean run(RobotContext robotContext) {
            return follower != null && follower.isBusy();
        }
    }
}
