package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class RunIntakeAuto extends CommandBase {
    private final IntakeSubsystem intake;

    public RunIntakeAuto(IntakeSubsystem intake) {
        this.intake = intake;

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.extendIntake();
    }

    @Override
    public void execute() {
            intake.runIntake(Constants.kIntakePercent);
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }
    
}
