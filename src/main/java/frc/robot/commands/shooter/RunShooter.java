package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotState;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.VisionTargetingParameters;

public class RunShooter extends CommandBase {
    private final ShooterSubsystem shooter;

    public RunShooter(ShooterSubsystem shooter) {
        this.shooter = shooter;

        addRequirements(shooter);
        SmartDashboard.putNumber("HoodPosSet", 0.0);
    }

    @Override
    public void initialize() {
        shooter.resetProfiler();

    }

    @Override
    public void execute() {
        VisionTargetingParameters params = RobotState.getInstance().getTargetingParameters(Robot.getTimestamp());
        shooter.runFlywheel();
        shooter.updateHood(params.getRangeToTargetM());
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
    }
}
