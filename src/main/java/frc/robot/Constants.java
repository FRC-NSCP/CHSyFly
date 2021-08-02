package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.util.Units;

public class Constants {
    public static final double kDt = 0.01; // Loop cycle time, in seconds

    // Robot geomertry
    // Length and width of robot (bumper to bumper)
    public static final double kRobotLengthMeters = Units.inchesToMeters(34.0);
    public static final double kRobotWidthMeters = Units.inchesToMeters(38.75);
    public static final double kRobotMassKg = 60.0;
    // Theoretical MOI, TODO replace this with measured value
    public static final double kRobotMOI = 1.0 / 12.0 * kRobotMassKg * (kRobotLengthMeters * kRobotLengthMeters + kRobotWidthMeters * kRobotWidthMeters);

    // Drive constants
    public static final double kDriveGearRatio = (64.0 / 12.0) * (40.0 / 24.0);
    public static final double kDriveWheelRadiusMeters = Units.inchesToMeters(3.0);
    public static final double kDriveTrackwidthMeters = Units.inchesToMeters(30.472);
    public static final double kDriveEmpTrackwidthMeters = Units.inchesToMeters(30.472);
    public static final double kTrackScrubFactor = kDriveTrackwidthMeters / kDriveEmpTrackwidthMeters;
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(Constants.kDriveEmpTrackwidthMeters);

    public static void main(String[] args) {
        System.out.println(kRobotMOI);
    }
}
