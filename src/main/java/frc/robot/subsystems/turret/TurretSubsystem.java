package frc.robot.subsystems.turret;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotState;
import org.team401.robot2020.control.turret.TurretController;

public class TurretSubsystem extends SubsystemBase {
    private TurretIO io;

    private Rotation2d angle;
    private double velocity;


    public TurretSubsystem(TurretIO io) {
        this.io = io;

    }

    @Override
    public void periodic() {
        angle = new Rotation2d(io.getPositionRadians());
        velocity = io.getVelocityRadiansPerSec();
        RobotState.getInstance().recordTurretObservations(Robot.getTimestamp(), angle, velocity);
    }
}
