package frc.robot;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import frckit.tools.pathview.TrajectoryVisualizer;
import frckit.util.GeomUtil;

import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

public class DriveTrajectories {
    /**
     * Generates a trajectory for the drive to follow
     * @param waypoints Input poses, in inches
     * @return A trajectory object, in meters
     */
    private static Trajectory gen(Pose2d... waypoints) {
        List<Pose2d> waypointsM = Arrays.stream(waypoints).map(GeomUtil::inchesToMeters).collect(Collectors.toList());
        return TrajectoryGenerator.generateTrajectory(waypointsM, Constants.kDriveTrajectoryConfig);
    }

    public static final Trajectory straightTrajectory = gen(
            new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0)),
            new Pose2d(12.0 * 6, -12.0 * 3, Rotation2d.fromDegrees(0)),
            new Pose2d(12.0 * 8, -12.0 * 3, Rotation2d.fromDegrees(0))
    );

    public static void main(String[] args) {
        TrajectoryVisualizer visualizer = new TrajectoryVisualizer(100, List.of(straightTrajectory), Constants.kDriveTrackwidthMeters, List.of());
        visualizer.start();
    }
}
