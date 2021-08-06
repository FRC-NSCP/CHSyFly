package frc.robot;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.util.Units;
import frckit.util.GeomUtil;
import org.team401.util.PolynomialRegression;

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
    public static final double kDriveWheelRadiusMeters = Units.inchesToMeters(2.9889324420536927806492191837982);
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
    public static final double kKickerPercent = 1.0;
    public static final double kFeederInPercent = 0.8;
    public static final double kHopperLeftPercent = 0.5;
    public static final double kHopperRightPercent = 0.6;
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
    public static final double kTurretHomingTimeSec = 0.2;
    public static final double kTurretHomingVelocityThresholdRadPerSec = Math.toRadians(1.0); // 1 deg/s
    public static final double kTurretGearRatio = 217.0 / 18.0;
    public static final double kTurretLookaheadSeconds = 0.7; // Amount to look ahead to account for vehicle velocity


    // Vision constants
    public static final Rotation2d kVisionHorizontalPlaneToLens = Rotation2d.fromDegrees(29.1);
    public static final double kVisionLensHeightM = Units.inchesToMeters(21.5);
    public static final double kVisionGoalHeightM = Units.inchesToMeters(98.0);
    public static final Pose2d kTurretToCamera = GeomUtil.inchesToMeters(new Pose2d(6.5, 0.0, GeomUtil.IDENTITY_ROTATION));


    // Shooter constants
    public static final double kShooterKs = 0.154;
    public static final double kShooterKv = 0.0182;
    public static final double kShooterKa = 0.00151;
    public static final double kShooterKp = 0.15;
    public static final double kShooterKd = 15.0;
    public static final double kShooterSpeed = 4750.0 * (2 * Math.PI / 60.0); // RPM -> rad / s
    public static final double kShooterDejamSpeed = -1000.0 * (2 * Math.PI / 60.0);
    public static final double KShooterAccel = kShooterSpeed; // Get to speed in 1 sec
    public static final double kHoodHomingVolts = -6.0;
    public static final double kHoodHomingThreshold = 5.0;
    public static final double kHoodHomingTime = 0.2;
    public static final double kHoodMin = 0.0;
    public static final double kHoodMax = 3.7;
    public static final double kHoodKp = 1.3;
    public static final double kHoodKd = 0.0;
    public static final double kShooterReadySpeedPercent = 0.03;

    private static final double shooterOffset = kVehicleToTurret.getX();
    public static final PolynomialRegression kHoodLut = new PolynomialRegression(
            new double[][]{
                    {Units.inchesToMeters(80) + shooterOffset, 2.75},
                    {Units.inchesToMeters(100) + shooterOffset, 3.2},
                    {Units.inchesToMeters(120) + shooterOffset, 3.5},
                    {Units.inchesToMeters(140) + shooterOffset, 3.57},
                    {Units.inchesToMeters(160) + shooterOffset, 3.55},
                    {Units.inchesToMeters(180) + shooterOffset, 3.47},
                    {Units.inchesToMeters(200) + shooterOffset, 3.5},
                    {Units.inchesToMeters(220) + shooterOffset, 3.6},
                    {Units.inchesToMeters(240) + shooterOffset, 3.7}
            },
            3
    );
}
