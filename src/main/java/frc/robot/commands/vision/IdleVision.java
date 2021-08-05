package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.vision.VisionSubsystem;

public class IdleVision extends CommandBase {
    private final VisionSubsystem vision;

    public IdleVision(VisionSubsystem vision) {
        this.vision = vision;

        addRequirements(vision);
    }

    @Override
    public void initialize() {
        vision.ledOff();
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
