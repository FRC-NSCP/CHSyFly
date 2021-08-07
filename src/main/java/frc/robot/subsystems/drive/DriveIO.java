package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj.geometry.Rotation2d;

/**
 * IO interface for the drive subsystem
 */
public interface DriveIO {
    /**
     * Update method which should be called on every loop cycle
     */
    void update();

    /**
     * Sets the voltage applied to the drivetrain motors
     * @param leftVolts The voltage to apply to the left motor
     * @param rightVolts The voltage to apply to the right motor
     */
    void setVoltage(double leftVolts, double rightVolts);

    void setVelocity(double leftRadPerSec, double rightRadPerSec, double leftFF, double rightFf);

    void setGains(double Kp, double Kd);

    void setBrake(boolean brake);

    double getLeftPositionRadians();
    double getRightPositionRadians();

    double getLeftVelocityRadPerSec();
    double getRightVelocityRadPerSec();

    double getHeadingRadians();

    void resetEncodersAndGyro();
}
