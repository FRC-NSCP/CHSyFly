package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class ClimbIOSim implements ClimbIO {
    boolean leftLockPosition, rightLockPosition = Constants.kClimbLock;
    double leftPower, rightPower = 0;

    public ClimbIOSim() {
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
        String leftLockStr =  leftLockPosition == Constants.kClimbLock ? "LOCKED,   " : "Unlocked,   ";
        String rightLockStr = rightLockPosition == Constants.kClimbLock ? "LOCKED" : "Unlocked";
        SmartDashboard.putString("Climb Locks", "Left: " + leftLockStr + "Right: " + rightLockStr);

        String leftPowerStr = String.format("%.3f", leftPower);
        String rightPowerStr = String.format("%.3f", rightPower);
        SmartDashboard.putString("Climb Power", "Left: " + leftPowerStr + ",    Right: " + rightPowerStr);
    }

    
}
