package frc.robot;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.util.Units;
import frckit.util.GeomUtil;

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

    // Intake constants
    public static final double kIntakePercent = 0.5;
    public static final double kSpitOutPercent = -0.5;
    public static final boolean kIntakeRetracted = false;
    public static final boolean kIntakeExtended = true;

    //Climb constants
    public static final boolean kClimbLock = false;
    public static final boolean kClimbUnlock = true;
    public static final double kClimberThreshold = 0.2;

    //Feeder constants
    public static final double kFeederInPercent = 0.7;
    public static final double kFeederOutPercent = -0.7;
    

    //Turret constants
    public static final Pose2d kVehicleToTurret = GeomUtil.inchesToMeters(new Pose2d(-6.25, 0.0, Rotation2d.fromDegrees(180.0)));
    public static final double kTurretKp = 70.0;
    public static final double kTurretKd = 1.0;
    public static final double kTurretTrackingKp = 30.0;
    public static final double kTurretTrackingKd = 0.1;
    public static final double kTurretKs = 0.161;
    public static final double kTurretKv = 1.9;
    public static final double kTurretKa = 0.0139;
    public static final double kTurretVelocityConstraintRadPerSec = 40.0 * (2 * Math.PI / 60.0); // RPM -> rad / s
    public static final double kTurretAccelConstraintRadPerSecPerSec = kTurretVelocityConstraintRadPerSec * 4; // rad / s / s
    public static final double kTurretRapidThresholdDistance = Math.toRadians(20.0); // Distance to switch to rapid mode instead of tracking
    public static final double kTurretLowerLimitAngleAbsoluteRad = Math.toRadians(-50.0);
    public static final double kTurretUpperLimitAngleAbsoluteRad = Math.toRadians(50.0);
    public static final double kTurretHardStopAngleAbsoluteRad = Math.toRadians(75.50736247119816 + 2.872443836405518);
    public static final double kTurretHomingVoltage = 2.0;
    public static final double kTurretHomingTimeSec = 0.1;
    public static final double kTurretHomingVelocityThresholdRadPerSec = Math.toRadians(1.0); // 1 deg/s
    public static final double kTurretGearRatio = 217.0 / 18.0;
    public static final double kTurretLookaheadSeconds = 0.7; // Amount to look ahead to account for vehicle velocity


    // Vision constants
    public static final Rotation2d kVisionHorizontalPlaneToLens = Rotation2d.fromDegrees(29.1);
    public static final double kVisionLensHeightM = Units.inchesToMeters(21.5);
    public static final double kVisionGoalHeightM = Units.inchesToMeters(98.0);
    public static final Pose2d kTurretToCamera = GeomUtil.inchesToMeters(new Pose2d(6.5, 0.0, GeomUtil.IDENTITY_ROTATION));
}
