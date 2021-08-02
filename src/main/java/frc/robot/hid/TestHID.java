package frc.robot.hid;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Test HID setup, using A logitech Dual Action controller (what I have on hand) to control all functions.
 * This setup may not be complete, as I will add and remove code as needed to map functions that I'm testing
 * to different buttons.
 */
public class TestHID implements HID {
    private final Joystick testGamepad = new Joystick(0);

    private Button testButton = new Button(() -> testGamepad.getRawButton(1));

    @Override
    public double getDriveTranslation() {
        return -testGamepad.getRawAxis(1); // Left stick Y (inverted)
    }

    @Override
    public double getDriveRotation() {
        return testGamepad.getRawAxis(2); // Right stick X
    }

    @Override
    public Button getTestButton() {
        return testButton;
    }
}
