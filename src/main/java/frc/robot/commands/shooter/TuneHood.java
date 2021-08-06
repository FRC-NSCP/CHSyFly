package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class TuneHood extends CommandBase {
    private final ShooterSubsystem shooter;
    private final double setpoint;

    public TuneHood(ShooterSubsystem shooter, double setpoint) {
        this.shooter = shooter;
        this.setpoint = setpoint;

        addRequirements(shooter);

        SmartDashboard.putNumber("HoodKp", Constants.kHoodKp);
        SmartDashboard.putNumber("HoodKd", Constants.kHoodKd);
    }

    @Override
    public void initialize() {
        shooter.setHoodGains(
                SmartDashboard.getNumber("HoodKp", Constants.kHoodKp),
                SmartDashboard.getNumber("HoodKd", Constants.kHoodKd)
        );
    }

    @Override
    public void execute() {
        shooter.updateHood(setpoint);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setHoodVoltage(0);
    }
}
