package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.*;
import edu.wpi.first.wpilibj.Talon;
import frc.robot.Constants;

public class ShooterIOReal implements ShooterIO {
    private final TalonFX leader = new TalonFX(9);
    private final TalonFX follower = new TalonFX(10);
    private final CANSparkMax hoodSpark = new CANSparkMax(5, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANEncoder hoodEncoder = hoodSpark.getEncoder();
    private final CANPIDController hoodPID = hoodSpark.getPIDController();
    private final CANSparkMax kicker = new CANSparkMax(6, CANSparkMaxLowLevel.MotorType.kBrushless);

    public ShooterIOReal() {
        // Set up leader/follower
        follower.follow(leader);
        follower.setInverted(TalonFXInvertType.OpposeMaster);
        leader.configVoltageCompSaturation(12.0);
        leader.enableVoltageCompensation(true);
        leader.configNeutralDeadband(0);
        follower.configNeutralDeadband(0);
        leader.config_kP(0, Constants.kShooterKp);
        leader.config_kD(0, Constants.kShooterKd);

        hoodSpark.setSmartCurrentLimit(40);
        hoodSpark.setInverted(true);
        hoodPID.setP(Constants.kHoodKp);
        hoodPID.setD(Constants.kHoodKd);

        kicker.setSmartCurrentLimit(20);
    }

    @Override
    public void setShooterVoltage(double volts) {
        leader.set(ControlMode.PercentOutput, volts / 12.0);
    }

    @Override
    public double getVelocityRadPerSec() {
        return leader.getSelectedSensorVelocity() / 2048.0 * 10.0 * 2.0 * Math.PI;
    }

    @Override
    public double getPositionRadians() {
        return leader.getSelectedSensorPosition() / 2048.0 * 2.0 * Math.PI;
    }

    @Override
    public void setVelocity(double velocity, double ffVolts) {
        double ticksPer100ms = velocity * 2048.0 / (10.0 * 2.0 * Math.PI);
        leader.set(ControlMode.Velocity, ticksPer100ms, DemandType.ArbitraryFeedForward, ffVolts / 12.0);
    }

    @Override
    public void setHoodVoltage(double volts) {
        hoodSpark.setVoltage(volts);
    }

    @Override
    public double getHoodPosition() {
        return hoodEncoder.getPosition();
    }

    @Override
    public double getHoodVelocity() {
        return hoodEncoder.getVelocity();
    }

    @Override
    public void setHoodPosition(double position) {
        hoodPID.setReference(position, ControlType.kPosition);
    }

    @Override
    public void setKickerPower(double percent) {
        kicker.set(percent);
    }

    @Override
    public void setHoodGains(double Kp, double Kd) {
        hoodPID.setP(Kp);
        hoodPID.setD(Kd);
    }

    @Override
    public void zeroHood() {
        hoodEncoder.setPosition(0);
    }
}
