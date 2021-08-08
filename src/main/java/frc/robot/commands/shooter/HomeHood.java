package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.ShooterSubsystem;


/**
 * Homes the turret.  This command should be run as non-interruptable to make sure it finishes
 */
public class HomeHood extends CommandBase {
    private final Timer timer = new Timer();
    private final ShooterSubsystem shooter;

    public HomeHood(ShooterSubsystem shooter) {
        this.shooter = shooter;

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        timer.stop();
        timer.reset();
        shooter.setHoodHomed(false);
    }

    @Override
    public void execute() {
        shooter.setHoodVoltage(Constants.kHoodHomingVolts);
        if (Math.abs(shooter.getHoodVelocity()) < Constants.kHoodHomingThreshold) {
            timer.start(); // Count the time that the turret has been stopped
        } else {
            timer.stop(); // Stop and reset the count
            timer.reset();
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.resetHoodPosition();
        shooter.setHoodHomed(true);
        shooter.setHoodVoltage(0);
        System.out.println("HOOD HOMING DONE");
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(Constants.kHoodHomingTime);
    }
}
