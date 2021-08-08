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

    private static Trajectory genM(Pose2d... waypoints) {
        List<Pose2d> waypointsM = Arrays.stream(waypoints).collect(Collectors.toList());
        return TrajectoryGenerator.generateTrajectory(waypointsM, Constants.kDriveTrajectoryConfig);
    }

    private static Trajectory genRev(Pose2d... waypoints) {
        List<Pose2d> waypointsM = Arrays.stream(waypoints).map(GeomUtil::inchesToMeters).collect(Collectors.toList());
        return TrajectoryGenerator.generateTrajectory(waypointsM, Constants.kDriveTrajectoryConfigReverse);
    }

    private static Trajectory genMRev(Pose2d... waypoints) {
        List<Pose2d> waypointsM = Arrays.stream(waypoints).collect(Collectors.toList());
        return TrajectoryGenerator.generateTrajectory(waypointsM, Constants.kDriveTrajectoryConfigReverse);
    }


    public static final Trajectory straightTrajectory = gen(
            new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0)),
            new Pose2d(12.0 * 2, 0, Rotation2d.fromDegrees(0))
    );

    public static final Trajectory eightBallCollectTrajectory = genM(
            new Pose2d(3, 6.75, Rotation2d.fromDegrees(0)),
            new Pose2d(5.375, 7.5, Rotation2d.fromDegrees(0)),
            new Pose2d(9.5, 7.5, Rotation2d.fromDegrees(0))
    );

    public static final Trajectory eightBallReturnTrajectory = genMRev(
            new Pose2d(9.5, 7.5, Rotation2d.fromDegrees(0)),
            new Pose2d(5.375, 7.5, Rotation2d.fromDegrees(0)),
            new Pose2d(3, 6.75, Rotation2d.fromDegrees(15))
    );

    public static final Trajectory stealCollectTrajectory = genM(
            new Pose2d(3, 0.692, Rotation2d.fromDegrees(0)),
            new Pose2d(6.113, 0.692, Rotation2d.fromDegrees(0))
    );

    public static final Trajectory stealReturnTrajectory = genMRev(
            new Pose2d(6.113, 0.692, Rotation2d.fromDegrees(0)),
            new Pose2d(3, 5.157, Rotation2d.fromDegrees(-45))
    );

    public static void main(String[] args) {
        TrajectoryVisualizer visualizer = new TrajectoryVisualizer(100, List.of(stealCollectTrajectory, stealReturnTrajectory), Constants.kDriveTrackwidthMeters, List.of());
        visualizer.start();
    }
}
