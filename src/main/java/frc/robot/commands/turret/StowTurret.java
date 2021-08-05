package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.turret.TurretSubsystem;

/**
 * Stows the turret (sends it to the zero position).
 */
public class StowTurret extends CommandBase {
    private final TurretSubsystem turret;

    public StowTurret(TurretSubsystem turret) {
        this.turret = turret;

        addRequirements(turret);
    }

    @Override
    public void initialize() {
        turret.startAbsoluteControl();
    }

    @Override
    public void execute() {
        turret.updateAbsoluteControl(0);
    }

    @Override
    public void end(boolean interrupted) {
        turret.setVoltage(0);
    }
}
