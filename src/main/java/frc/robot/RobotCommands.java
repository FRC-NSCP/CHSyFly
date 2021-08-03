package frc.robot;

import frc.robot.commands.drive.DriveOperatorControl;
import frc.robot.hid.HID;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class RobotCommands {
    private HID hid;
    private DriveSubsystem drive;
    private IntakeSubsystem intake;

    public final DriveOperatorControl driveOperatorControl;

    public RobotCommands(HID hid, DriveSubsystem drive, IntakeSubsystem intake) {
        this.drive = drive;
        this.intake = intake;

        driveOperatorControl = new DriveOperatorControl(drive, hid);
    }
}
