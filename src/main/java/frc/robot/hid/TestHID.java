package frc.robot.hid;

import edu.wpi.first.wpilibj.Joystick;

/**
 * Test HID setup, using A logitech Dual Action controller (what I have on hand) to control all functions.
 * This setup may not be complete, as I will add and remove code as needed to map functions that I'm testing
 * to different buttons.
 */
public class TestHID implements HID {
    private final Joystick testGamepad = new Joystick(0);

    @Override
    public double getDriveTranslation() {
        return -testGamepad.getRawAxis(1); // Left stick Y (inverted)
    }

    @Override
    public double getDriveRotation() {
        return testGamepad.getRawAxis(2); // Right stick X
    }
}
