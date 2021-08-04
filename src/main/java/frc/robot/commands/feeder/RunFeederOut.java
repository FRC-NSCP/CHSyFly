package frc.robot.commands.feeder;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.hid.HID;
import frc.robot.subsystems.feeder.FeederSubsystem;

public class RunFeederOut extends CommandBase {
    private final FeederSubsystem feeder;
    private final HID hid;

    public RunFeederOut(FeederSubsystem feeder, HID hid) {
        this.hid = hid;
        this.feeder = feeder;

        addRequirements(feeder);
    }

    @Override
    public void execute() {
        if (hid.unJamHopper().get())
            feeder.runAll(Constants.kFeederOutPercent);
        else
            feeder.stop();
    }

    @Override
    public void end(boolean interrupted) {
        feeder.stop();
    }
}
