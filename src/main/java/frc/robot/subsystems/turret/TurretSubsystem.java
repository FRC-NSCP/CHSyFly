package frc.robot.subsystems.turret;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotState;
import frc.robot.commands.turret.HomeTurret;
import org.team401.robot2020.control.turret.TurretController;

public class TurretSubsystem extends SubsystemBase {
    private final TurretIO io;

    private final TurretController controller = TurretController.createFromJava(
            Constants.kTurretKp,
            Constants.kTurretKd,
            Constants.kTurretTrackingKp,
            Constants.kTurretTrackingKd,
            Constants.kTurretKs,
            Constants.kTurretKv,
            Constants.kTurretKa,
            Constants.kTurretVelocityConstraintRadPerSec,
            Constants.kTurretAccelConstraintRadPerSecPerSec,
            Constants.kTurretRapidThresholdDistance,
            Constants.kTurretLowerLimitAngleAbsoluteRad,
            Constants.kTurretUpperLimitAngleAbsoluteRad,
            this::getAbsolutePositionRadians,
            this::getVelocityRadPerSec,
            this::setVoltage,
            Constants.kDt
    );

    private boolean isHomed = false;
    public boolean getHomed() {
        return isHomed;
    }
    public void setHomed(boolean homed) {
        this.isHomed = homed;
    }

    private final HomeTurret homeCommand = new HomeTurret(this);


    private Rotation2d angle;
    private double velocity;

    public TurretSubsystem(TurretIO io) {
        this.io = io;

    }

    SimpleMotorFeedforward ff = new SimpleMotorFeedforward(Constants.kTurretKs, Constants.kTurretKv, Constants.kTurretKa);

    @Override
    public void periodic() {
        angle = new Rotation2d(getAbsolutePositionRadians());
        velocity = io.getVelocityRadiansPerSec();

        if (!isHomed && DriverStation.getInstance().isOperatorControlEnabled()) {
            homeCommand.schedule(false);
            return;
        }
        RobotState.getInstance().recordTurretObservations(Robot.getTimestamp(), angle, velocity);

        SmartDashboard.putNumber("TurretVelSet", controller.getSetpointVelocityRadPerSec());
        SmartDashboard.putNumber("TurretVel", velocity);
    }

    public double getAbsolutePositionRadians() {
        return io.getPositionRadians() + Constants.kTurretHardStopAngleAbsoluteRad;
    }

    public Rotation2d getAngle() {
        return angle;
    }

    public double getVelocityRadPerSec() {
        return velocity;
    }

    public void setVoltage(double voltage) {
        io.setVoltage(voltage);
    }

    public void resetPosition() {
        io.zeroEncoder();
    }

    public void setRapidGains(double Kp, double Kd) {
        controller.setRapidGains(Kp, Kd);
    }

    public void startAbsoluteControl() {
        controller.enterAbsoluteAngle();
    }

    public void updateAbsoluteControl(double setpoint) {
        controller.updateAbsoluteAngle(setpoint);
    }

    public void startAngleControl() {
        controller.enterAngle();
    }

    public void updateAngleControl(Rotation2d angle, double feedVelocityRadPerSec) {
        controller.updateAngle(angle, feedVelocityRadPerSec);
    }
}
