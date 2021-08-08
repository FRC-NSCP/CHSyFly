package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotState;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.VisionTargetingParameters;

public class RunShooter extends CommandBase {
    private final ShooterSubsystem shooter;

    public enum ShootMode {
        ODOM,
        LIMP_FIX,
        WALL_FIX
    }

    private final ShootMode mode;

    public RunShooter(ShooterSubsystem shooter) {
        this(shooter, ShootMode.ODOM);
    }

    public RunShooter(ShooterSubsystem shooter, ShootMode mode) {
        this.shooter = shooter;
        this.mode = mode;

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        System.out.println("STARTING RUN SHOOTER");
        shooter.resetProfiler();

    }

    @Override
    public void execute() {
        VisionTargetingParameters params = RobotState.getInstance().getTargetingParameters(Robot.getTimestamp());
        shooter.runFlywheel();
        switch (mode) {
            case ODOM:
                shooter.updateHood(Constants.kHoodLut.predict(params.getRangeToTargetM()));
                break;
            case LIMP_FIX:
                shooter.updateHood(Constants.kHoodLut.predict(Constants.kLimpShootDistanceM));
                break;
            case WALL_FIX:
                shooter.updateHood(0);
                break;
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
    }
}
