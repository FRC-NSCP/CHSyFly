package frc.robot.commands.turret;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotState;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.subsystems.vision.VisionTargetingParameters;

public class TurretTrackTarget extends CommandBase {
    private TurretSubsystem turret;

    public TurretTrackTarget(TurretSubsystem turret) {
        this.turret = turret;

        addRequirements(turret);
    }

    @Override
    public void initialize() {
        turret.startAngleControl();
    }

    @Override
    public void execute() {
        VisionTargetingParameters params = RobotState.getInstance().getTargetingParameters(Robot.getTimestamp());
        Rotation2d targetAngle = new Rotation2d(turret.getAbsolutePositionRadians() + params.getTurretError().getRadians());

        SmartDashboard.putString("TargetingError", params.getTurretError().toString());
        turret.updateAngleControl(targetAngle, params.getTurretFeedVelRadPerSec());
    }

    @Override
    public void end(boolean interrupted) {
        turret.setVoltage(0.0);
    }
}
