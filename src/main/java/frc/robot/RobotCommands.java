package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.climb.RunClimbers;
import frc.robot.commands.drive.DriveOperatorControl;
import frc.robot.commands.feeder.FeedShooter;
import frc.robot.commands.feeder.LoadTower;
import frc.robot.commands.feeder.RunFeederIn;
import frc.robot.commands.feeder.RunFeederOut;
import frc.robot.commands.intake.RunIntakeTeleop;
import frc.robot.commands.intake.StowIntake;
import frc.robot.commands.turret.CharacterizeTurret;
import frc.robot.commands.turret.StowTurret;
import frc.robot.commands.turret.TuneTurret;
import frc.robot.hid.HID;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;

public class RobotCommands {
    private HID hid;
    private DriveSubsystem drive;
    private IntakeSubsystem intake;
    private ClimbSubsystem climb;
    private FeederSubsystem feeder;

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

    public RobotCommands(HID hid, DriveSubsystem drive, IntakeSubsystem intake, ClimbSubsystem climb, FeederSubsystem feeder, TurretSubsystem turret) {
        this.hid = hid;
        this.drive = drive;
        this.intake = intake;

        this.climb = climb;
        this.feeder = feeder;

        driveOperatorControl = new DriveOperatorControl(drive, hid);
        stowIntake = new StowIntake(intake, hid);
        runIntakeTeleop = new RunIntakeTeleop(intake, hid);
        runClimbers = new RunClimbers(climb, hid);
        feedShooter = new FeedShooter(feeder);
        loadTower = new LoadTower(feeder);
        feederIn = new RunFeederIn(feeder);
        feederOut = new RunFeederOut(feeder, hid);

        characterizeTurret = new CharacterizeTurret(turret);
        tuneTurret0 = new TuneTurret(turret, 0.0);
        tuneTurret90 = new TuneTurret(turret, Math.toRadians(-90));
        stowTurret = new StowTurret(turret);
        SmartDashboard.putData("TuneTurret0", tuneTurret0);
        SmartDashboard.putData("TuneTurret90", tuneTurret90);

        bindHID();
    }

    /**
     * Binds HID buttons to command actions
     */
    private void bindHID() {
        hid.toggleIntakeButton().toggleWhenPressed(runIntakeTeleop);
        hid.runHopper().whenHeld(feederIn);
        hid.unJamHopper().whenHeld(feederOut);
        characterizeTurret.schedule();
    }
}
