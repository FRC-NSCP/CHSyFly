package frc.robot.commands.feeder;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.feeder.FeederSubsystem;

public class RunFeederOut extends CommandBase {
    private final FeederSubsystem feeder;

    public RunFeederOut(FeederSubsystem feeder) {
        this.feeder = feeder;

        addRequirements(feeder);
    }

    @Override
    public void execute() {
        feeder.runAll(Constants.kFeederOutPercent);
    }

    @Override
    public void end(boolean interrupted) {
        feeder.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
