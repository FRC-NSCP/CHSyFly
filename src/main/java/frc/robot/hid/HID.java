package frc.robot.hid;

import edu.wpi.first.wpilibj2.command.button.Button;

/**
 * Interface for the human interface.  Provides methods to get axis values and Trigger objects to bind to commands.
 */
public interface HID {
    double getDriveTranslation();
    double getDriveRotation();

    Button getTestButton();
}
