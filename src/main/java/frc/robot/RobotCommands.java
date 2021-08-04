package frc.robot;

import frc.robot.commands.climb.RunClimbers;
import frc.robot.commands.drive.DriveOperatorControl;
import frc.robot.commands.feeder.FeedShooter;
import frc.robot.commands.feeder.LoadTower;
import frc.robot.commands.feeder.RunFeederIn;
import frc.robot.commands.feeder.RunFeederOut;
import frc.robot.commands.intake.RunIntakeTeleop;
import frc.robot.commands.intake.StowIntake;
import frc.robot.hid.HID;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;

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

    public RobotCommands(HID hid, DriveSubsystem drive, IntakeSubsystem intake, ClimbSubsystem climb, FeederSubsystem feeder) {
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

        bindHID();
    }

    /**
     * Binds HID buttons to command actions
     */
    private void bindHID() {
        hid.toggleIntakeButton().toggleWhenPressed(runIntakeTeleop);
        hid.runHopper().whenPressed(loadTower);
        hid.unJamHopper().whenHeld(feederOut);
    }
}
