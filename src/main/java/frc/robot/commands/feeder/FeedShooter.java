package frc.robot.commands.feeder;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.feeder.FeederSubsystem;

public class FeedShooter extends CommandBase {
    private final FeederSubsystem feeder;

    public FeedShooter(FeederSubsystem feeder) {
        this.feeder = feeder;
    }

    @Override
    public void execute() {
        feeder.runAll(Constants.kFeederInPercent);
    }
    
}
