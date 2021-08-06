package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class RunShooter extends CommandBase {
    private final ShooterSubsystem shooter;

    public RunShooter(ShooterSubsystem shooter) {
        this.shooter = shooter;

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.resetProfiler();
    }

    @Override
    public void execute() {
        shooter.updateProfiler();
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setVoltage(0.0);
    }
}
