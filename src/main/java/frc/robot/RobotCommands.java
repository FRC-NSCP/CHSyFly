package frc.robot;

import frc.robot.commands.climb.RunClimbers;
import frc.robot.commands.drive.DriveOperatorControl;
import frc.robot.commands.intake.RunIntakeTeleop;
import frc.robot.commands.intake.StowIntake;
import frc.robot.hid.HID;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class RobotCommands {
    private HID hid;
    private DriveSubsystem drive;
    private IntakeSubsystem intake;
    private ClimbSubsystem climb;

    public final DriveOperatorControl driveOperatorControl;
    public final StowIntake stowIntake;
    public final RunIntakeTeleop runIntakeTeleop;
    public final RunClimbers runClimbers;

    public RobotCommands(HID hid, DriveSubsystem drive, IntakeSubsystem intake, ClimbSubsystem climb) {
        this.hid = hid;
        this.drive = drive;
        this.intake = intake;
        this.climb = climb;

        driveOperatorControl = new DriveOperatorControl(drive, hid);
        stowIntake = new StowIntake(intake);
        runIntakeTeleop = new RunIntakeTeleop(intake, hid);
        runClimbers = new RunClimbers(climb, hid);

        bindHID();
    }

    /**
     * Binds HID buttons to command actions
     */
    private void bindHID() {
        hid.togglePositionButton().toggleWhenPressed(runIntakeTeleop);
        hid.runIntakeButton().whenPressed(runIntakeTeleop);
        hid.runOuttakeButton().whenPressed(runIntakeTeleop);
    }
}
