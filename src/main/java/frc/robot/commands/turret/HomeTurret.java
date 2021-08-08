package frc.robot.commands.turret;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.turret.TurretSubsystem;


/**
 * Homes the turret.  This command should be run as non-interruptable to make sure it finishes
 */
public class HomeTurret extends CommandBase {
    private final Timer timer = new Timer();
    private final TurretSubsystem turret;

    public HomeTurret(TurretSubsystem turret) {
        this.turret = turret;

        addRequirements(turret);
    }

    @Override
    public void initialize() {
        timer.stop();
        timer.reset();
        turret.setHomed(false);
    }

    @Override
    public void execute() {
        turret.setVoltage(Constants.kTurretHomingVoltage);
        if (Math.abs(turret.getVelocityRadPerSec()) < Constants.kTurretHomingVelocityThresholdRadPerSec) {
            timer.start(); // Count the time that the turret has been stopped
        } else {
            timer.stop(); // Stop and reset the count
            timer.reset();
        }
    }

    @Override
    public void end(boolean interrupted) {
        turret.resetPosition();
        turret.setVoltage(0.0);
        turret.setHomed(true);
        System.out.println("TURRET HOMING DONE");
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(Constants.kTurretHomingTimeSec); // This command is done when the turret is homed
    }
}
