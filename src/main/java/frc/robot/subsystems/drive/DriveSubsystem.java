package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hid.HID;

public class DriveSubsystem extends SubsystemBase {
    private DriveIO io;

    public DriveSubsystem(DriveIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.update();
    }

    public void stop() {
        io.setVoltage(0, 0);
    }

    public void setPercentOut(double left, double right) {
        io.setVoltage(left * 12, right * 12);
    }
}
