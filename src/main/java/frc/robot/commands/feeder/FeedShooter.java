package frc.robot.commands.feeder;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class FeedShooter extends CommandBase {
    private final FeederSubsystem feeder;
    private final ShooterSubsystem shooter;

    public FeedShooter(FeederSubsystem feeder, ShooterSubsystem shooter) {
        this.feeder = feeder;
        this.shooter = shooter;

        addRequirements(feeder);
    }

    @Override
    public void execute() {
        if (shooter.readyToShoot()) {
            feeder.runAll(Constants.kFeederInPercent);
        } else {
            feeder.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        feeder.stop();
    }
}
