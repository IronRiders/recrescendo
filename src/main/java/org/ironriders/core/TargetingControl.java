package org.ironriders.core;

import java.util.ArrayList;
import java.util.List;

import org.ironriders.drive.DriveSubsystem;
import org.ironriders.lib.DriverRequest;
import org.ironriders.lib.DriverRequest.AlignTargetingMode;
import org.ironriders.lib.DriverRequest.LauncherTargetingMode;
import org.ironriders.lib.DriverRequest.PriorityMode;
import org.ironriders.lib.Utils;
import org.ironriders.lib.field.FieldPositions;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/*
 * Class holding state for targeting points with launcher and drive angles.
 */
public class TargetingControl {
    private static DriverRequest request = new DriverRequest(PriorityMode.DRIVER_PRIORITY,
            AlignTargetingMode.LAUNCHER, LauncherTargetingMode.HUB);

    private static DriverRequest lastDriverRequest = request;

    private static double alignTarget;

    private static List<Pose2d> points = new ArrayList<Pose2d>();

    public static void init() {
        for (Pose2d point : FieldPositions.Zones.PASSING_POINTS) {
            points.add(FieldPositions.prepareMetersPose(point));
        }

        RobotContainer.scoringZone.printPolygon();
        RobotContainer.passingZone.printPolygon();
    }

    public static void receiveRequest(DriverRequest driverRequest) {
        lastDriverRequest = request;
        request = driverRequest;
    }

    public static void revert() {
        request = lastDriverRequest;
        update();
    }

    public static void revertToSafeDefaults() {
        lastDriverRequest = request;
        request = new DriverRequest(PriorityMode.DRIVER_PRIORITY,
                AlignTargetingMode.LAUNCHER, LauncherTargetingMode.HUB);
    }

    public static void targetHubInternal() {
        new DriverRequest(PriorityMode.LAUNCHER_PRIORITY,
                AlignTargetingMode.LAUNCHER, LauncherTargetingMode.HUB).send("target hub internal");
    }

    public static Command targetHub() {
        return Commands.runOnce(() -> targetHubInternal());
    }

    public static void targetPassingInternal() {
        new DriverRequest(PriorityMode.LAUNCHER_PRIORITY,
                AlignTargetingMode.LAUNCHER, LauncherTargetingMode.PASSING).send("target passing internal");
    }

    public static Command targetPassing() {
        return Commands.runOnce(() -> targetPassingInternal());
    }

    /*
     * Update the state of everything relevant to targeting. Should be called every
     * tick.
     */
    public static void update() {
        alignTarget = getAlignTarget();

        DriveSubsystem.setRotationGoal(alignTarget);

        switch (request.r_priorityMode) {
            default:
            case DRIVER_PRIORITY:
                DriveSubsystem.setPIDRotationControl(false);

                return;

            case ALIGN_PRIORITY:
            case LAUNCHER_PRIORITY:
                DriveSubsystem.setPIDRotationControl(true);

                return;
        }
    }

    private static double getAlignTarget() {
        switch (request.r_alignTargetingMode) {
            default:
            case LAUNCHER:
                return Utils.getAngleToPoint(DriveSubsystem.getPose(),
                        RobotContainer.scoringZone.closestPoint());

            case OUTPOST:
                return 180;

            case BUMP:
                return Math.toDegrees(
                        findClosest45DegreeAngleInRadians(DriveSubsystem.getRotation()));
        }
    }

    private static double findClosest45DegreeAngleInRadians(double angle) {
        angle = angle % (2 * Math.PI);
        if (angle < 0) {
            angle += 2 * Math.PI;
        }

        for (Double i = 0.25; i <= 2; i += 0.5) {
            if (Math.abs(angle - (i * Math.PI)) < 0.25 * Math.PI) {
                return i * Math.PI;
            }
        }

        return 0.25 * Math.PI;
    }
}