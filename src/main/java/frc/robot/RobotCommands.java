package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.climb.RunClimbers;
import frc.robot.commands.drive.CharacterizeDrive;
import frc.robot.commands.drive.DisabledCoastCommand;
import frc.robot.commands.drive.DriveOperatorControl;
import frc.robot.commands.drive.FollowTrajectoryCommand;
import frc.robot.commands.feeder.*;
import frc.robot.commands.intake.RunIntakeAuto;
import frc.robot.commands.intake.RunIntakeTeleop;
import frc.robot.commands.intake.StowIntake;
import frc.robot.commands.shooter.CharacterizeShooter;
import frc.robot.commands.shooter.HomeHood;
import frc.robot.commands.shooter.RunShooter;
import frc.robot.commands.shooter.TuneHood;
import frc.robot.commands.turret.*;
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

public class RobotCommands {
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

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
    public final RunShooter runShooter, runShooterLimp, runShooterWall;

    public final CharacterizeDrive characterizeDrive;
    public final DisabledCoastCommand disabledCoastCommand;
    public final FollowTrajectoryCommand driveStraightTest;
    public final FollowTrajectoryCommand driveEightBallAutoCollectTrajectory;
    public final FollowTrajectoryCommand driveEightBallAutoReturnTrajectory;
    public final FollowTrajectoryCommand driveStealAutoCollectTrajectory;
    public final FollowTrajectoryCommand driveStealAutoReturnTrajectory;
    public final FollowTrajectoryCommand driveBackFromWall;


    public final Command trackAndRunShooter;

    public final Command runWallSequence;

    // Autos
    public final Command runEightBallAuto;
    public final Command runStealAuto;

    public RobotCommands(HID hid, DriveSubsystem drive, IntakeSubsystem intake, ClimbSubsystem climb, FeederSubsystem feeder, TurretSubsystem turret, VisionSubsystem vision, ShooterSubsystem shooter) {
        this.hid = hid;
        this.drive = drive;
        this.intake = intake;
        this.climb = climb;
        this.feeder = feeder;
        this.vision = vision;
        this.shooter = shooter;

        driveOperatorControl = new DriveOperatorControl(drive, hid);
        stowIntake = new StowIntake(intake, shooter, hid);
        runIntakeTeleop = new RunIntakeTeleop(intake, hid);
        runClimbers = new RunClimbers(climb, hid);
        feedShooter = new FeedShooter(feeder, shooter);
        loadTower = new LoadTower(feeder);
        feederIn = new RunFeederIn(feeder);
        feederOut = new RunFeederOut(feeder, hid);

        characterizeTurret = new CharacterizeTurret(turret);
        tuneTurret0 = new TuneTurret(turret, 0.0);
        tuneTurret90 = new TuneTurret(turret, Math.toRadians(-90));
        stowTurret = new StowTurret(turret);
        turretTrackTarget = new TurretTrackTarget(turret);

        idleVision = new IdleVision(vision);
        runVisionTracking = new RunVisionTracking(vision);

        characterizeShooter = new CharacterizeShooter(shooter);
        runShooter = new RunShooter(shooter, RunShooter.ShootMode.ODOM);
        runShooterLimp = new RunShooter(shooter, RunShooter.ShootMode.LIMP_FIX);
        runShooterWall = new RunShooter(shooter, RunShooter.ShootMode.WALL_FIX);

        characterizeDrive = new CharacterizeDrive(drive);
        disabledCoastCommand = new DisabledCoastCommand(drive);
        driveStraightTest = new FollowTrajectoryCommand(drive, DriveTrajectories.straightTrajectory, true);
        trackAndRunShooter = new RunShooter(shooter).alongWith(new RunVisionTracking(vision), new TurretTrackTarget(turret));

        driveEightBallAutoCollectTrajectory = new FollowTrajectoryCommand(drive, DriveTrajectories.eightBallCollectTrajectory, true);
        driveEightBallAutoReturnTrajectory = new FollowTrajectoryCommand(drive, DriveTrajectories.eightBallReturnTrajectory, false);

        driveStealAutoCollectTrajectory = new FollowTrajectoryCommand(drive, DriveTrajectories.stealCollectTrajectory, true);
        driveStealAutoReturnTrajectory = new FollowTrajectoryCommand(drive, DriveTrajectories.stealReturnTrajectory, false);

        driveBackFromWall = new FollowTrajectoryCommand(drive, DriveTrajectories.wallReverseTrajectory, true);

        runEightBallAuto = new SequentialCommandGroup(
                new InstantCommand(() -> RobotState.getInstance().forceRobotPose(DriveTrajectories.eightBallCollectTrajectory.getInitialPose())),
                new HomeHood(shooter),
                new ParallelCommandGroup(
                        new RunShooter(shooter),
                        new SequentialCommandGroup(
                                new HomeTurret(turret),
                                new ParallelCommandGroup(
                                        new RunVisionTracking(vision),
                                        new TurretTrackTarget(turret),
                                        new SequentialCommandGroup(
                                                new WaitCommand(0.25),
                                                new FeedShooter3(feeder, shooter).withTimeout(2),
                                                driveEightBallAutoCollectTrajectory.deadlineWith(new RunIntakeAuto(intake)),
                                                driveEightBallAutoReturnTrajectory.deadlineWith(new StowIntake(intake, shooter, hid)),
                                                new FeedShooter(feeder, shooter).alongWith(new StowIntake(intake, shooter, hid))
                                        )
                                )
                        )
                )
        );

        runStealAuto = new SequentialCommandGroup(
                new InstantCommand(() -> RobotState.getInstance().forceRobotPose(DriveTrajectories.stealCollectTrajectory.getInitialPose())),
                new HomeHood(shooter),
                new ParallelCommandGroup(
                        new RunShooter(shooter),
                        new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                        new HomeTurret(turret),
                                        driveStealAutoCollectTrajectory.deadlineWith(new RunIntakeAuto(intake))
                                ),
                                new ParallelCommandGroup(
                                        new RunVisionTracking(vision),
                                        new TurretTrackTarget(turret),
                                        new SequentialCommandGroup(
                                                driveStealAutoReturnTrajectory.deadlineWith(new StowIntake(intake, shooter, hid)),
                                                new FeedShooter(feeder, shooter).alongWith(new RunIntakeAuto(intake))
                                        )
                                )
                        )
                ));

        runWallSequence = new RunShooter(shooter, RunShooter.ShootMode.WALL_FIX).alongWith(driveBackFromWall.andThen(new FeedShooter(feeder, shooter)));

                autoChooser.setDefaultOption("Eight Ball", runEightBallAuto);
                autoChooser.addOption("Steal", runStealAuto);


        SmartDashboard.putData("Auto Selector", autoChooser);

        bindHID();
    }

    public Command getAutoCommand() {
        return autoChooser.getSelected();
    }

    /**
     * Binds HID buttons to command actions
     */
    private void bindHID() {
        hid.toggleIntakeButton().toggleWhenPressed(runIntakeTeleop);
        //hid.runHopper().whenHeld(feederIn);
        hid.unJamHopper().whenHeld(feederOut, false);
        hid.shootBall().whileHeld(trackAndRunShooter).whileHeld(feedShooter); // Whileheld to resume if interrupted
        hid.limpShoot().whileHeld(runShooterLimp).whileHeld(feedShooter).whileHeld(stowTurret);
        hid.wallShoot().whenHeld(runWallSequence);
        hid.centerTurret().whenPressed(stowTurret).whenHeld(runVisionTracking);
        hid.resetDrive().whenPressed(drive::reset);
    }
}
