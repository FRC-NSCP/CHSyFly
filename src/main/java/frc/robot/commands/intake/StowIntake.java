package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.hid.HID;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class StowIntake extends CommandBase {
    private final IntakeSubsystem intake;
    private final ShooterSubsystem shooter;
    private final HID hid;

    public StowIntake(IntakeSubsystem intake, ShooterSubsystem shooter, HID hid) {
        this.intake = intake;
        this.hid = hid;
        this.shooter = shooter;

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.retractIntake();
        intake.stop();
    }

    @Override
    public void execute() {
        if (hid.runIntakeButton()) {
            intake.runIntake(Constants.kIntakePercent);
        } else if (hid.runOuttakeButton()) {
            intake.runIntake(Constants.kSpitOutPercent);
        } else if (shooter.isShooting()) {
            intake.runIntake(Constants.kIntakeShootPercent);
        } else {
            intake.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {return false;}
}
