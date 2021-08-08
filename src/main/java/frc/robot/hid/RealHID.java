package frc.robot.hid;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RealHID implements HID {
    Joystick leftStick = new Joystick(0);
    Joystick rightStick = new Joystick(1);
    Joystick controller = new Joystick(5);

    private final Button toggleIntake = new JoystickButton(controller, 1);
    private final Button centerTurret = new JoystickButton(rightStick, 4);
    private final Button startShooter = new JoystickButton(controller, 7);
    private final Button runHopper = new JoystickButton(controller, 6);
    private final Button unjamHopper = new JoystickButton(controller, 8);
    private final Button shootBall = new JoystickButton(rightStick, 2);
    private final Button resetDrive = new JoystickButton(rightStick, 5);


    @Override
    public Button resetDrive() {
        return resetDrive;
    }

    @Override
    public double getDriveTranslation() {
        return -leftStick.getY() * Math.abs(leftStick.getY());
    }

    @Override
    public double getDriveRotation() {
        return rightStick.getX() * Math.abs(rightStick.getX());
    }

    @Override
    public double getLeftClimber() {
        return controller.getRawAxis(1);
    }

    @Override
    public double getRightClimber() {
        return controller.getRawAxis(3);
    }

    @Override
    public boolean runIntakeButton() {
        return controller.getPOV() == 180;
    }

    @Override
    public boolean runOuttakeButton() {
        return controller.getPOV() == 0;
    }

    @Override
    public Button toggleIntakeButton() {
        return toggleIntake;
    }

    @Override
    public Button shootBall() {
        return shootBall;
    }

    @Override
    public Button startShoot() {
        return shootBall;
    }

    @Override
    public Button revUpShooter() {
        return startShooter;
    }

    @Override
    public Button unJamHopper() {
        return unjamHopper;
    }

    @Override
    public Button runHopper() {
        return runHopper;
    }

    @Override
    public Button centerTurret() {
        return centerTurret;
    }
    
}
