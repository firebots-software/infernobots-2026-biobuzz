package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.net.InetAddress;
import java.net.UnknownHostException;

public class CameraTesting {

    public static void main(String[] args) {
        System.out.println("==========================================");
        System.out.println("  Limelight Desktop Localizer Test Tool  ");
        System.out.println("==========================================");

        // 1. Determine Limelight IP address
        String targetIpStr = "172.29.0.1"; // Default FTC USB Limelight IP
        if (args.length > 0 && args[0] != null && !args[0].isEmpty()) {
            targetIpStr = args[0];
        }

        InetAddress inetAddress = resolveLimelightIp(targetIpStr);
        if (inetAddress == null) {
            System.err.println("Could not resolve IP for Limelight. Exiting.");
            return;
        }

        System.out.println("Connecting to Limelight at: " + inetAddress.getHostAddress());

        // 2. Instantiate Limelight3A and LimelightLocalizer
        Limelight3A limelight = new Limelight3A(null, "limelight", inetAddress);
        LimelightLocalizer localizer = new LimelightLocalizer(limelight);

        // Register shutdown hook for clean stop
        Runtime.getRuntime().addShutdownHook(new Thread(() -> {
            System.out.println("\nStopping Limelight localizer...");
            localizer.stop();
        }));

        // 3. Start localizer polling thread
        localizer.start();

        System.out.println("Limelight polling started. Waiting for data from Limelight...");

        // 4. Main test loop
        int loopCount = 0;
        while (!Thread.currentThread().isInterrupted()) {
            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                break;
            }

            loopCount++;

            boolean connected = limelight.isConnected();
            long timeSinceLastUpdate = limelight.getTimeSinceLastUpdate();

            if (!connected && loopCount % 20 == 0) {
                System.out.printf("[%d] Waiting for Limelight data... (Connected: %b, Last update: %d ms ago)%n",
                        loopCount, connected, timeSinceLastUpdate);
            } else if (connected) {
                LLResult result = limelight.getLatestResult();
                Pose3D rawPose = localizer.getLimelightPose();
                Pose correctedPose = localizer.getCorrectedPoseInInches();

                System.out.println("------------------------------------------");
                System.out.printf("Loop #%d | Status: CONNECTED | Last Update: %d ms ago%n",
                        loopCount, timeSinceLastUpdate);

                if (result != null) {
                    System.out.printf("  Valid Result: %b | Pipeline Index: %d%n",
                            result.isValid(),
                            (int) result.getPipelineIndex());
                }

                if (rawPose != null) {
                    System.out.printf("  Raw BotPose (3D): X=%.3f m, Y=%.3f m, Z=%.3f m%n",
                            rawPose.getPosition().x,
                            rawPose.getPosition().y,
                            rawPose.getPosition().z);
                } else {
                    System.out.println("  Raw BotPose (3D): [No AprilTag/Fiducial target locked]");
                }

                if (correctedPose != null) {
                    System.out.printf("  Corrected Pedro Pose: X=%.2f in, Y=%.2f in, Heading=%.2f rad (%.1f deg)%n",
                            correctedPose.getX(),
                            correctedPose.getY(),
                            correctedPose.getHeading(),
                            Math.toDegrees(correctedPose.getHeading()));
                } else {
                    System.out.println("  Corrected Pedro Pose: [Null]");
                }
            }
        }

        localizer.stop();
        System.out.println("Test ended.");
    }

    private static InetAddress resolveLimelightIp(String preferredIpStr) {
        try {
            return InetAddress.getByName(preferredIpStr);
        } catch (UnknownHostException e) {
            System.err.println("Unknown host: " + preferredIpStr);
            try {
                return InetAddress.getByName("limelight.local");
            } catch (UnknownHostException ex) {
                return null;
            }
        }
    }
}
