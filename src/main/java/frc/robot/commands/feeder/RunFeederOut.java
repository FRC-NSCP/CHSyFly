package frc.robot.commands.feeder;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.hid.HID;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class RunFeederOut extends CommandBase {
    private final FeederSubsystem feeder;
    private final ShooterSubsystem shooter;
    private final HID hid;

    public RunFeederOut(FeederSubsystem feeder, ShooterSubsystem shooter, HID hid) {
        this.hid = hid;
        this.feeder = feeder;
        this.shooter = shooter;

        addRequirements(feeder, shooter);
    }

    @Override
    public void initialize() {
        shooter.resetProfiler();
    }

    @Override
    public void execute() {
        if (hid.unJamHopper().get()) {
            feeder.runAll(Constants.kFeederOutPercent);
            shooter.updateDejam();
        } else {
            feeder.stop();
            shooter.setFlywheelVoltage(0.0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        feeder.stop();
        shooter.stop();
    }
}
