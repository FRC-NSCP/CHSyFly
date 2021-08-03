package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.hid.HID;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class RunIntakeTeleop extends CommandBase {
    private final IntakeSubsystem intake;
    private final HID hid;
    
    public RunIntakeTeleop(IntakeSubsystem intake, HID hid) {
        this.intake = intake;
        this.hid = hid;

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.extendIntake();
    }

    @Override
    public void execute() {
        if (hid.runIntakeButton().get())
            intake.runIntake(Constants.kIntakePercent);
        else if (hid.runOuttakeButton().get())
            intake.runIntake(Constants.kSpitOutPercent);
        else
            intake.runIntake(0);
    }

    @Override
    public void end(boolean interrupted) {
        intake.runIntake(0);
    }

    @Override
    public boolean isFinished() {return false;}
}