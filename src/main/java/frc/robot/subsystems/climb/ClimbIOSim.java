package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ClimbIOSim implements ClimbIO {
    boolean locked = true;
    boolean unlocked = !locked;

    boolean leftLockPosition, rightLockPosition;
    double leftPower, rightPower;

    public ClimbIOSim() {
        leftLockPosition = rightLockPosition = locked;
        leftPower = rightPower = 0;

        updateDash();
    }

    @Override
    public void setLeftLock(boolean lockPosition) {
        leftLockPosition = lockPosition;
        updateDash();
    }

    @Override
    public void setRightLock(boolean lockPosition) {
        rightLockPosition = lockPosition;
        updateDash();
    }

    @Override
    public void setLeftPower(double climbPower) {
        leftPower = climbPower;
        updateDash();
    }

    @Override
    public void setRightPower(double climbPower) {
        rightPower = climbPower;
        updateDash();
    }

    private void updateDash() {
        String leftStr =  leftLockPosition ? "LOCKED,   " : "Unlocked,   ";
        String rightStr = rightLockPosition ? "LOCKED" : "Unlocked";
        SmartDashboard.putString("Climb Locks", "Left: " + leftStr + "Right: " + rightStr);
        SmartDashboard.putString("Climb Powers", "Left: " + leftPower + ",    Right: " + rightPower);
    }

    
}
