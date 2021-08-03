package frc.robot.subsystems.climb;

public interface ClimbIO {
    void setLeftLock(boolean lockPosition);
    void setRightLock(boolean lockPosition);
    void setLeftPower(double climbPower);
    void setRightPower(double climbPower);
}
