package frc.robot;

import frc.robot.commands.drive.DriveOperatorControl;
import frc.robot.hid.HID;
import frc.robot.subsystems.drive.DriveSubsystem;

public class RobotCommands {
    private HID hid;
    private DriveSubsystem drive;

    public final DriveOperatorControl driveOperatorControl;

    public RobotCommands(HID hid, DriveSubsystem drive) {
        this.drive = drive;

        driveOperatorControl = new DriveOperatorControl(drive, hid);
    }
}
