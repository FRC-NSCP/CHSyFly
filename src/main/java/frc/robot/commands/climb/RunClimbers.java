package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.hid.HID;
import frc.robot.subsystems.climb.ClimbSubsystem;

public class RunClimbers extends CommandBase {
    private final ClimbSubsystem climb;
    private final HID hid;

    public RunClimbers(ClimbSubsystem climb, HID hid) {
        this.climb = climb;
        this.hid = hid;

        addRequirements(climb);
    }

    @Override
    public void initialize() {
        climb.stop();
    }

    @Override
    public void execute() {
        double leftPowerValue = hid.getLeftClimber();
        double rightPowerValue = hid.getRightClimber();

        boolean runLeft = Math.abs(leftPowerValue) > Constants.kClimberThreshold;
        boolean runRight = Math.abs(rightPowerValue) > Constants.kClimberThreshold;

        climb.setLeftLock(runLeft ? Constants.kClimbUnlock : Constants.kClimbLock);
        climb.setLeftPower(runLeft ? leftPowerValue : 0.0);

        climb.setRightLock(runRight ? Constants.kClimbUnlock : Constants.kClimbLock);
        climb.setRightPower(runRight ? rightPowerValue : 0.0);
    }

    @Override
    public void end(boolean interrupted) {
        climb.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
