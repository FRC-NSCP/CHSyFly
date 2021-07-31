package frc.robot.subsystems.drive;

/**
 * IO interface for the drive subsystem
 */
public interface DriveIO {
    /**
     * Sets the voltage applied to the drivetrain motors
     * @param leftVolts The voltage to apply to the left motor
     * @param rightVolts The voltage to apply to the right motor
     */
    void setVoltage(double leftVolts, double rightVolts);

    void getLeftPositionRadians();
    void getRightPositionRadians();

    void getLeftVelocityRadPerSec();
    void getRightVelocityRadPerSec();
}
