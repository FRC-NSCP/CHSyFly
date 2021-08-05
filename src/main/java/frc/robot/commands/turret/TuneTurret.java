package frc.robot.commands.turret;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.turret.TurretSubsystem;

public class TuneTurret extends CommandBase {
    private final TurretSubsystem turret;
    private final double setpoint;

    public TuneTurret(TurretSubsystem turret, double setpoint) {
        this.turret = turret;
        addRequirements(turret);
        this.setpoint = setpoint;
        SmartDashboard.putNumber("TurretKp", Constants.kTurretKp);
        SmartDashboard.putNumber("TurretKd", Constants.kTurretKd);
    }

    @Override
    public void initialize() {
        turret.setRapidGains(
                SmartDashboard.getNumber("TurretKp", Constants.kTurretKp),
                SmartDashboard.getNumber("TurretKd", Constants.kTurretKd)
        );

        turret.startAbsoluteControl();
    }

    @Override
    public void execute() {
        turret.updateAbsoluteControl(setpoint);
    }

    @Override
    public void end(boolean interrupted) {
        turret.setVoltage(0.0);
    }
}
