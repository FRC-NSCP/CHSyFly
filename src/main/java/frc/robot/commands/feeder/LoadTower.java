package frc.robot.commands.feeder;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.feeder.FeederSubsystem;

public class LoadTower extends CommandBase {
    private final FeederSubsystem feeder;

    public LoadTower(FeederSubsystem feeder) {
        this.feeder = feeder;

        addRequirements(feeder);
    }

    @Override
    public void execute() {
        if (!feeder.seesBall())
            feeder.runLower(Constants.kFeederInPercent);
        else
            feeder.stop();
    }

    @Override
    public void end(boolean interrupted) {
        feeder.stop();
    }
}
