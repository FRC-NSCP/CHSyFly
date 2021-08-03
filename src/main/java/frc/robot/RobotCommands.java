package frc.robot;

import frc.robot.commands.drive.DriveOperatorControl;
import frc.robot.commands.intake.RunIntakeTeleop;
import frc.robot.commands.intake.StowIntake;
import frc.robot.hid.HID;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class RobotCommands {
    private HID hid;
    private DriveSubsystem drive;
    private IntakeSubsystem intake;

    public final DriveOperatorControl driveOperatorControl;
    public final StowIntake stowIntake;
    public final RunIntakeTeleop runIntakeTeleop;

    public RobotCommands(HID hid, DriveSubsystem drive, IntakeSubsystem intake) {
        this.drive = drive;
        this.intake = intake;

        driveOperatorControl = new DriveOperatorControl(drive, hid);
        stowIntake = new StowIntake(intake);
        runIntakeTeleop = new RunIntakeTeleop(intake, hid);

        bindHID();
    }

    /**
     * Binds HID buttons to command actions
     */
    private void bindHID() {
        hid.togglePositionButton().toggleWhenPressed(runIntakeTeleop);
    }
}
