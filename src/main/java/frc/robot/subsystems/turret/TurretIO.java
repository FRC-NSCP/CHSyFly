package frc.robot.subsystems.turret;

public interface TurretIO {
    void zeroEncoder();
    double getPositionRadians();
    double getVelocityRadiansPerSec();
    void setVoltage(double volts);
}
