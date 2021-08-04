package frc.robot.commands.feeder;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.feeder.FeederSubsystem;

public class RunFeederIn extends CommandBase {
    private final FeederSubsystem feeder;
    
    public RunFeederIn(FeederSubsystem feeder) {
        this.feeder = feeder;
    }

    @Override
    public void initialize() {
        feeder.stop();
    }

    @Override
    public void execute() {
        feeder.runLower(Constants.kFeederInPercent);
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
