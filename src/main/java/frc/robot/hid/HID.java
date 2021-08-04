package frc.robot.hid;

import edu.wpi.first.wpilibj2.command.button.Button;

/**
 * Interface for the human interface.  Provides methods to get axis values and Trigger objects to bind to commands.
 */
public interface HID {
    double getDriveTranslation();
    double getDriveRotation();
    
    double getLeftClimber();
    double getRightClimber();

    boolean runIntakeButton();
    boolean runOuttakeButton();
    Button toggleIntakeButton();

    Button centerTurret();
    Button shootBall();
    Button startShoot();

    Button revUpShooter();

    Button unJamHopper();
    Button runHopper();
}
