package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbSubsystem extends SubsystemBase {
    private ClimbIO io;

    public ClimbSubsystem(ClimbIO io) {
        this.io = io;
    }

    public void setLeftLock(boolean lockPosition) {
        io.setLeftLock(lockPosition);
    }

    public void setRightLock(boolean lockPosition) {
        io.setRightLock(lockPosition);
    }

    public void setBothLocks(boolean lockPosition) {
        setLeftLock(lockPosition);
        setRightLock(lockPosition);
    }

    public void setLeftPower(double power) {
        io.setLeftPower(power);
    }

    public void setRightPower(double power) {
        io.setRightPower(power);
    }

    public void setBothPower(double power) {
        setLeftPower(power);
        setRightPower(power);
    }

    public void stop() {
        setBothPower(0.0);
        setBothLocks(Constants.kClimbLock);
    }
    
}
