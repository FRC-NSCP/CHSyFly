package frc.robot.subsystems.shooter;

public interface ShooterIO {
    void setVoltage(double volts);
    double getVelocityRadPerSec();
    double getPositionRadians();
    void setVelocity(double velocity, double ffVolts);

    void setHoodVoltage(double volts);
    double getHoodPosition();
    double getHoodVelocity();
    void setHoodPosition(double position);
    void setHoodGains(double Kp, double Kd);
    void zeroHood();
}
