package frc.robot.commands.feeder;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class FeedShooter extends CommandBase {
    private final FeederSubsystem feeder;
    private final ShooterSubsystem shooter;

    private final Timer feedTimer = new Timer();

    public FeedShooter(FeederSubsystem feeder, ShooterSubsystem shooter) {
        this.feeder = feeder;
        this.shooter = shooter;

        addRequirements(feeder);
    }

    @Override
    public void initialize() {
        feedTimer.reset();
        feedTimer.start();
    }

    @Override
    public void execute() {
        if (shooter.readyToShoot()) {
            feeder.runFeeder(Constants.kFeederInPercent);
            if (feedTimer.hasElapsed(Constants.kFeederLowerWaitTime)) {
                feeder.runFunnel(Constants.kHopperLeftPercent, Constants.kHopperRightPercent);
            } else {
                feeder.runFunnel(0, 0);
            }
        } else {
            feeder.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        feeder.stop();
    }
}
