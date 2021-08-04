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
    public double getLeftClimber() {
        return testGamepad.getRawAxis(3);
    }

    @Override
    public double getRightClimber() {
        return testGamepad.getRawAxis(4);
    }

    @Override
    public Button toggleIntakeButton() {
        return intakeOut;
    }

    @Override
    public Button shootBall() {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public boolean runIntakeButton() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public boolean runOuttakeButton() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public Button centerTurret() {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public Button startShoot() {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public Button revUpShooter() {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public Button unJamHopper() {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public Button runHopper() {
        // TODO Auto-generated method stub
        return null;
    }

}
