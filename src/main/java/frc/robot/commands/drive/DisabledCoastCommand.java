package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.DriveSubsystem;

public class DisabledCoastCommand extends CommandBase {
    private final DriveSubsystem drive;
    private final Timer timer = new Timer();

    public DisabledCoastCommand(DriveSubsystem drive) {
        this.drive = drive;

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(2.0);
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted) {
            drive.setCoast();
        }
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
