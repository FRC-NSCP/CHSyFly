package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {
    private final boolean locked = true;
    private final boolean unlocked = !locked;
    private ClimbIO io;

    public ClimbSubsystem(ClimbIO io) {
        this.io = io;
    }

    public void stop() {
        io.setLeftPower(0.0);
        io.setRightPower(0.0);
        io.setLeftLock(locked);
        io.setRightLock(locked);
    }
    
}
