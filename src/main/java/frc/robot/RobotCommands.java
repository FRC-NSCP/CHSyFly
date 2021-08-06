package frc.robot;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.climb.RunClimbers;
import frc.robot.commands.drive.CharacterizeDrive;
import frc.robot.commands.drive.DriveOperatorControl;
import frc.robot.commands.feeder.FeedShooter;
import frc.robot.commands.feeder.LoadTower;
import frc.robot.commands.feeder.RunFeederIn;
import frc.robot.commands.feeder.RunFeederOut;
import frc.robot.commands.intake.RunIntakeTeleop;
import frc.robot.commands.intake.StowIntake;
import frc.robot.commands.shooter.CharacterizeShooter;
import frc.robot.commands.shooter.RunShooter;
import frc.robot.commands.shooter.TuneHood;
import frc.robot.commands.turret.CharacterizeTurret;
import frc.robot.commands.turret.StowTurret;
import frc.robot.commands.turret.TuneTurret;
import frc.robot.commands.turret.TurretTrackTarget;
import frc.robot.commands.vision.IdleVision;
import frc.robot.commands.vision.RunVisionTracking;
import frc.robot.hid.HID;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frckit.util.GeomUtil;

import java.util.List;

public class RobotCommands {
    private final HID hid;
    private final DriveSubsystem drive;
    private final IntakeSubsystem intake;
    private final ClimbSubsystem climb;
    private final FeederSubsystem feeder;
    private final VisionSubsystem vision;
    private final ShooterSubsystem shooter;

    public final DriveOperatorControl driveOperatorControl;
    public final StowIntake stowIntake;
    public final RunIntakeTeleop runIntakeTeleop;
    public final RunClimbers runClimbers;
    public final FeedShooter feedShooter;
    public final LoadTower loadTower;
    public final RunFeederIn feederIn;
    public final RunFeederOut feederOut;

    public final CharacterizeTurret characterizeTurret;
    public final TuneTurret tuneTurret0, tuneTurret90;
    public final StowTurret stowTurret;
    public final TurretTrackTarget turretTrackTarget;

    public final IdleVision idleVision;
    public final RunVisionTracking runVisionTracking;

    public final CharacterizeShooter characterizeShooter;
    public final RunShooter runShooter;

    public final TuneHood tuneHood0, tuneHood1, tuneHood2, tuneHood3;

    public final CharacterizeDrive characterizeDrive;

    public final SequentialCommandGroup driveStraight;

    public RobotCommands(HID hid, DriveSubsystem drive, IntakeSubsystem intake, ClimbSubsystem climb, FeederSubsystem feeder, TurretSubsystem turret, VisionSubsystem vision, ShooterSubsystem shooter) {
        this.hid = hid;
        this.drive = drive;
        this.intake = intake;
        this.climb = climb;
        this.feeder = feeder;
        this.vision = vision;
        this.shooter = shooter;

        driveOperatorControl = new DriveOperatorControl(drive, hid);
        stowIntake = new StowIntake(intake, hid);
        runIntakeTeleop = new RunIntakeTeleop(intake, hid);
        runClimbers = new RunClimbers(climb, hid);
        feedShooter = new FeedShooter(feeder, shooter);
        loadTower = new LoadTower(feeder);
        feederIn = new RunFeederIn(feeder);
        feederOut = new RunFeederOut(feeder, shooter, hid);

        characterizeTurret = new CharacterizeTurret(turret);
        tuneTurret0 = new TuneTurret(turret, 0.0);
        tuneTurret90 = new TuneTurret(turret, Math.toRadians(-90));
        stowTurret = new StowTurret(turret);
        turretTrackTarget = new TurretTrackTarget(turret);
        SmartDashboard.putData("TuneTurret0", tuneTurret0);
        SmartDashboard.putData("TuneTurret90", tuneTurret90);
        SmartDashboard.putData("TurretTrackTarget", turretTrackTarget);

        idleVision = new IdleVision(vision);
        runVisionTracking = new RunVisionTracking(vision);
        SmartDashboard.putData("DebugRunVisionTracking", runVisionTracking);

        characterizeShooter = new CharacterizeShooter(shooter);
        runShooter = new RunShooter(shooter);
        SmartDashboard.putData("RunShooterDebug", runShooter);
        SmartDashboard.putData("FeedShooterDebug", feedShooter);

        tuneHood0 = new TuneHood(shooter, 0);
        tuneHood1 = new TuneHood(shooter, 1);
        tuneHood2 = new TuneHood(shooter, 2);
        tuneHood3 = new TuneHood(shooter, 3);
        SmartDashboard.putData("TuneHood0", tuneHood0);
        SmartDashboard.putData("TuneHood1", tuneHood1);
        SmartDashboard.putData("TuneHood2", tuneHood2);
        SmartDashboard.putData("TuneHood3", tuneHood3);

        //SmartDashboard.putData("AimAndRunShooter", runShooter.alongWith(turretTrackTarget, runVisionTracking));

        characterizeDrive = new CharacterizeDrive(drive);

        driveStraight = drive.createTrajectoryCommand(
                List.of(GeomUtil.IDENTITY_POSE, GeomUtil.inchesToMeters(new Pose2d(7.0 * 12, -3.0 * 12, GeomUtil.IDENTITY_ROTATION)))
        ).beforeStarting(() -> RobotState.getInstance().forceRobotPose(GeomUtil.IDENTITY_POSE));

        bindHID();
    }

    /**
     * Binds HID buttons to command actions
     */
    private void bindHID() {
        hid.toggleIntakeButton().toggleWhenPressed(runIntakeTeleop);
        hid.runHopper().whenHeld(feederIn);
        hid.unJamHopper().whenHeld(feederOut);
        hid.shootBall().whenHeld(feedShooter);
    }
}
