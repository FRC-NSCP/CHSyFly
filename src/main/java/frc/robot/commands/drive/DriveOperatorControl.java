package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.hid.HID;
import frc.robot.subsystems.drive.DriveSubsystem;

public class DriveOperatorControl extends CommandBase {
    private final HID hid;
    private final DriveSubsystem drive;

    public DriveOperatorControl(DriveSubsystem drive, HID hid) {
        this.hid = hid;
        this.drive = drive;

        addRequirements(drive);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        // Read joystick values
        double translation = hid.getDriveTranslation();
        double rotation = hid.getDriveRotation();
        drive.setPercentOut(translation + rotation, translation - rotation);
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }

    @Override
    public boolean isFinished() {
        return false; // Operator control enver ends on its own
    }
}
