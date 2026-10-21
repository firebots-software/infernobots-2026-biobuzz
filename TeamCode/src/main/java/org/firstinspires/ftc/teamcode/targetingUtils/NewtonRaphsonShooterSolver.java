package org.firstinspires.ftc.teamcode.targetingUtils;

import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Intermap;

public class NewtonRaphsonShooterSolver {

    private static final int MAX_ITERATIONS = 10;
    private static final double CONVERGENCE_TOLERANCE = 1e-4;
    private static final double DELTA_T = 1e-4;

    /**
     * Calculates targeting information for shooting on the move using the Newton-Raphson numerical method.
     *
     * @param robotPose     Current pose of the robot (x, y, heading)
     * @param robotVelocity Current velocity vector of the robot (vx, vy, omega)
     * @param targetPose    Position of the goal/target (x, y)
     * @param speedLut      Intermap mapping target distance to flywheel speed
     * @param tofLut        Intermap mapping target distance to projectile time of flight
     * @return ShootingTargetInfo containing aiming heading, required speed, virtual distance, and time of flight
     */
    public static ShootingTargetInfo calculateTargeting(
            Pose robotPose,
            Pose robotVelocity,
            Pose targetPose,
            Intermap speedLut,
            Intermap tofLut
    ) {
        double dx = targetPose.getX() - robotPose.getX();
        double dy = targetPose.getY() - robotPose.getY();
        double stationaryDist = Math.hypot(dx, dy);

        // Initial time of flight estimate
        double t = getTimeOfFlightForDistance(stationaryDist, speedLut, tofLut);

        // Newton-Raphson iteration to solve f(t) = t - TOF(d_effective(t)) = 0
        for (int i = 0; i < MAX_ITERATIONS; i++) {
            double fVal = evaluateF(t, robotPose, robotVelocity, targetPose, speedLut, tofLut);

            // Compute numerical derivative f'(t) = (f(t + delta) - f(t - delta)) / (2 * delta)
            double fPlus = evaluateF(t + DELTA_T, robotPose, robotVelocity, targetPose, speedLut, tofLut);
            double fMinus = evaluateF(t - DELTA_T, robotPose, robotVelocity, targetPose, speedLut, tofLut);
            double fPrime = (fPlus - fMinus) / (2.0 * DELTA_T);

            if (Math.abs(fPrime) < 1e-6) {
                break;
            }

            double tNext = t - fVal / fPrime;

            // Clamp tNext to stay physical (> 0)
            tNext = Math.max(0.01, tNext);

            if (Math.abs(tNext - t) < CONVERGENCE_TOLERANCE) {
                t = tNext;
                break;
            }

            t = tNext;
        }

        // Calculate virtual target location given converged time of flight t
        double virtTargetX = targetPose.getX() - robotVelocity.getX() * t;
        double virtTargetY = targetPose.getY() - robotVelocity.getY() * t;
        Pose virtTargetPose = new Pose(virtTargetX, virtTargetY);

        double virtDx = virtTargetX - robotPose.getX();
        double virtDy = virtTargetY - robotPose.getY();
        double virtualDistance = Math.hypot(virtDx, virtDy);

        // Aiming heading angle
        double targetHeading = Math.atan2(virtDy, virtDx);

        // Required shooter flywheel speed
        double shooterSpeed = getSpeedForDistance(virtualDistance, speedLut);

        return new ShootingTargetInfo(targetHeading, shooterSpeed, virtualDistance, t, virtTargetPose);
    }

    private static double evaluateF(
            double t,
            Pose robotPose,
            Pose robotVelocity,
            Pose targetPose,
            Intermap speedLut,
            Intermap tofLut
    ) {
        // Effective virtual displacement vector from robot to target compensating for robot motion
        double virtX = targetPose.getX() - robotVelocity.getX() * t - robotPose.getX();
        double virtY = targetPose.getY() - robotVelocity.getY() * t - robotPose.getY();
        double effectiveDist = Math.hypot(virtX, virtY);

        double expectedTOF = getTimeOfFlightForDistance(effectiveDist, speedLut, tofLut);
        return t - expectedTOF;
    }

    private static double getTimeOfFlightForDistance(double dist, Intermap speedLut, Intermap tofLut) {
        if (tofLut != null && !tofLut.isEmpty()) {
            return tofLut.get(dist);
        }
        double speed = getSpeedForDistance(dist, speedLut);
        if (speed > 0) {
            return dist / speed;
        }
        return Constants.Shooter.defaultTimeOfFlight;
    }

    private static double getSpeedForDistance(double dist, Intermap speedLut) {
        if (speedLut != null && !speedLut.isEmpty()) {
            return speedLut.get(dist);
        }
        return Constants.Shooter.defaultSpeed;
    }
}
