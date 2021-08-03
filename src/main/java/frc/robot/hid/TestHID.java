package frc.robot.hid;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * Test HID setup, using A logitech Dual Action controller (what I have on hand) to control all functions.
 * This setup may not be complete, as I will add and remove code as needed to map functions that I'm testing
 * to different buttons.
 */
public class TestHID implements HID {
    private final Joystick testGamepad = new Joystick(0);

    private final Button inButton = new JoystickButton(testGamepad, 1);
    private final Button outButton = new JoystickButton(testGamepad, 2);
    private final Button intakeOut = new JoystickButton(testGamepad, 3);

    @Override
    public double getDriveTranslation() {
        return -testGamepad.getRawAxis(1); // Left stick Y (inverted)
    }

    @Override
    public double getDriveRotation() {
        return testGamepad.getRawAxis(2); // Right stick X
    }

    @Override
    public Button runIntakeButton() {
        return inButton;
    }

    @Override
    public Button runOuttakeButton() {
        return outButton;
    }

    @Override
    public Button togglePositionButton() {
        return intakeOut;
    }

}
