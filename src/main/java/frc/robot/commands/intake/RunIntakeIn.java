package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.hid.HID;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class RunIntakeIn extends CommandBase {
    private final HID hid;
    private final IntakeSubsystem intake;
    
    public RunIntakeIn(IntakeSubsystem intake, HID hid) {
        this.hid = hid;
        this.intake = intake;

        addRequirements(intake);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {return false;}
}
